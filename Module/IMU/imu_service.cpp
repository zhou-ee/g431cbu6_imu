/**
 * @file    imu_service.cpp
 * @brief   IMU 采集 + VQF 姿态融合服务实现（任务层）。
 *
 * ============================ 架构说明 ============================
 * 单个采集任务按 1 ms 固定周期顺序完成：
 *   ① imu_base_drv::read()  —— 内部自行完成"状态查询 → 等 DMA 完成中断唤醒
 *                              → 数据读取 → 再次唤醒 → 解包"，数据源对上层不可见
 *   ② vqf_t::update()       —— 按样本自带时间戳合并两路，做姿态融合
 *   ③ 按 VQF 报告的消费量归还环形缓冲
 *   ④ 发布姿态快照、统计耗时、vTaskDelayUntil 对齐下一拍
 *
 * ---------------------------- 为什么是单任务 ----------------------------
 * 最初实现为"采集任务(HIGH) + 算法任务(NORMAL)"双任务，靠环形缓冲交接。实测
 * 出现三类无法回避的问题，全部源于并发：
 *   1) 算法任务处理途中被采集任务抢占并 push()，缓冲满会前移 head，而 at(i)
 *      的物理下标依赖 head —— 读到错位样本，时间戳不再单调，Butterworth 的
 *      dt 缓存被反复击穿（每样本重算 double 版 exp/cos/sin），VQF 慢一个量级。
 *   2) 在算法侧用 taskENTER_CRITICAL() 打包拷贝：该调用把 BASEPRI 提到
 *      configMAX_SYSCALL_INTERRUPT_PRIORITY，而 SPI/DMA 中断优先级恰好等于它，
 *      会被一并屏蔽；采集任务此时正等 DMA 完成信号量，只能等到超时
 *      （实测采集周期从 1000 Hz 掉到 254 Hz）。
 *   3) 双缓冲交接若计数清零时机不当，会让某区在旧 count 上继续累加，
 *      样本顺序错乱、时间戳倒退，VQF 返回 PARAM_ERROR（实测 err_stage=4）。
 *
 * 单任务串行时这些都不存在：read() 与 update() 之间没有别的写入者，环形缓冲
 * 的 head 在处理期间不会移动，样本天然按时间戳单调。本工程的测试工程
 * lsm6dsv_test 正是单任务结构，在 7680 Hz 下稳定运行，这里与其保持一致。
 *
 * 代价：VQF 的耗时计入 1 ms 周期内。实测单批（约 8 样本/路）在 -O2 下远小于
 * 预算；若后续加入更多算法导致超标，应通过降低 ODR 或分频送入融合解决，
 * 而不是重新拆成双任务。
 *
 * ---------------------------- 关于实时性 ----------------------------
 * read() 内部经 DMA 完成信号量让出 CPU，不轮询；SPI 传输期间任务处于阻塞态。
 * VQF 计算期间本任务占满 CPU，但它是最低耦合的做法：1 ms 周期内做完即返回。
 */

#include "imu_service.h"

#include "lsm6dsv16x_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_task.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <cmath>

/* ======================= 调试观测变量 ======================= */
extern "C" {
volatile int32_t  imu_service_init_result  = -1;
volatile int32_t  imu_service_vqf_ready    = 0;
volatile uint32_t imu_service_read_ok      = 0U;
volatile uint32_t imu_service_algo_rounds  = 0U;
volatile uint32_t imu_service_vqf_consumed = 0U;
volatile uint32_t imu_service_err_bus      = 0U;
volatile uint32_t imu_service_err_data     = 0U;
volatile uint32_t imu_service_err_timeout  = 0U;
volatile uint32_t imu_service_vqf_err      = 0U;
volatile uint32_t imu_service_vqf_warn     = 0U;
volatile int32_t  imu_service_vqf_status   = 0;
volatile uint32_t imu_service_vqf_us_max   = 0U;
volatile uint32_t imu_service_drop_samples = 0U;
volatile uint32_t imu_service_heap_free_min = 0U;
volatile float    imu_service_accel_mps2[3] = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_gyro_rps[3]   = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_quat[4]      = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float    imu_service_quat_norm    = 1.0f;
volatile float    imu_service_roll         = 0.0f;
volatile float    imu_service_pitch        = 0.0f;
volatile float    imu_service_yaw          = 0.0f;
volatile float    imu_service_bias_rps[3]  = {0.0f, 0.0f, 0.0f};
volatile uint32_t imu_service_rest         = 0U;
volatile float    imu_service_rest_t       = 0.0f;
}

namespace
{

constexpr uint32_t k_cpu_mhz = 170U;

/* ======================= 产品配置 =======================
 * ODR 取 1920 Hz 而非 7680 Hz：VQF 的姿态精度取决于陀螺积分的角度增量
 * 与加速度倾斜校正的时间常数（tau_acc=0.25 s，对应截止频率约 0.9 Hz），
 * 与采样率无关；1~2 kHz 远高于 VQF 官方推荐的 200~1000 Hz 上限。
 *
 * 7680 Hz 下每毫秒两路共 15.4 个样本，VQF 实测每样本约 110 µs（含零偏 KF 的
 * 3x3 矩阵求逆与多个 Butterworth 双精度递推），单拍耗时 1.6 ms 已超 1 ms 周期，
 * 会形成"越攒越多、越算越慢"的正反馈（实测跌到 330~390 Hz）。
 * 1920 Hz 下每毫秒两路共 3.8 个样本，单拍约 0.4 ms，留足 1 ms 预算。
 *
 * 若后续需要更高采样率（如做振动分析），应把 VQF 独立成低优先级任务并按
 * FIFO 批次节流，而不是提高本任务的 ODR。 */
const imu_cfg_t k_imu_cfg = {
    false,                  /* calibrate_flag ：关闭上电标定，零偏交给 VQF 静止估计 */
    500U,                   /* calibrate_times：1920 Hz 下约 260 ms */
    4U,                     /* acc_range      ：±4 g   */
    1920U,                  /* acc_freq       ：1920 Hz */
    2000U,                  /* gyro_range     ：±2000 dps */
    1920U,                  /* gyro_freq      ：1920 Hz */
    true,                   /* fifo_en        ：true=FIFO 批量；false=寄存器直读回退 */
    imu_direct_t::DIRECT_1, /* direct         ：按实际安装方向改这里 */
};

/* ======================= VQF 配置 =======================
 * 门限按本板静止实测噪声设定：驱动已做单位换算与安装方向映射，
 * 静止时角速度残余约 1 °/s 量级（零偏未扣除），故 rest_th_gyr 放到 5 °/s。
 * 加速度静止波动约 0.2~0.3 m/s²，rest_th_acc 放到 1.0 留余量。
 * bias_clip 必须 ≥ rest_th_gyr，否则低通值被提前截断，静止永不成立。 */
const pyro::vqf_t::config_t k_vqf_cfg = [] {
    pyro::vqf_t::config_t c;
    c.tau_acc      = 0.25f;  /* 加速度倾斜校正时间常数，s（默认 3.0 偏慢） */
    c.rest_min_t   = 1.0f;   /* 静止确认时间，s */
    c.rest_th_gyr  = 5.0f;   /* °/s */
    c.rest_th_acc  = 1.0f;   /* m/s² */
    c.bias_clip    = 5.0f;   /* °/s，与 rest_th_gyr 匹配 */
    c.motion_bias_est_enabled = false; /* 高采样率下关闭运动零偏估计，省算力 */
    c.rest_bias_est_enabled   = true;  /* 保留静止零偏估计 */
    return c;
}();

/* ======================= 板级绑定 ======================= */
/* 片选由产品装配层绑定（CubeMX 中 PA4 初始为高）。换芯片/换板只改这两行。 */
GPIO_TypeDef *const k_cs_port = GPIOA;
const uint16_t k_cs_pin = GPIO_PIN_4;

/* ======================= 总线与驱动装配 ======================= */
/* 换 SPI/I²C 只需换这一行；驱动不感知总线类型。
 * 第 4 参 dma_enable：true = 收发走 DMA，完成经中断通知并唤醒任务。 */
lsm6dsv16x_spi_t s_bus(&hspi1, k_cs_port, k_cs_pin, true);
lsm6dsv16x_drv_t s_imu(k_imu_cfg, s_bus);

/* VQF 实例：固定存储（约 1.6 KB），静态区分配，无动态内存。 */
pyro::vqf_t s_vqf;

/* 姿态快照：采集任务写、其他任务读。
 * 用版本号（seqlock）保证读方拿到的一定是完整的一次更新：
 *   写方：seq++（变奇数=更新中）→ 写结构体 → seq++（变偶数=稳定）
 *   读方：读 seq 为偶数 → 拷贝 → 再读 seq，未变则有效，否则重试 */
imu_attitude_t s_attitude{};
volatile uint32_t s_attitude_seq = 0U;

/* 四元数 → 欧拉角（ZYX 顺序，°）。仅用于对外快照与调试观测。 */
void quat_to_euler(const float q[4], float &roll, float &pitch, float &yaw)
{
    constexpr float rad_to_deg = 57.29577951308232f;
    const float w = q[0], x = q[1], y = q[2], z = q[3];

    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp) * rad_to_deg;

    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) { sinp = 1.0f; }
    if (sinp < -1.0f) { sinp = -1.0f; }
    pitch = std::asin(sinp) * rad_to_deg;

    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp) * rad_to_deg;
}

/* ======================= 采集 + 融合任务 ======================= */
class imu_task_t final : public pyro::task_base_t
{
  public:
    /* 栈 1024 词 = 4 KB：`_update_bias` 的调用链实测最大单帧 592 B（-O0），
     * 叠加 update → _update_acc → _update_bias → kf.correct 的嵌套接近 1.5 KB；
     * 本任务还同时承载 read() → SPI 中间件 → 驱动解包。
     * 早期给 512 词时栈溢出会踩坏相邻堆块，表现为 PendSV 恢复时 HardFault。 */
    imu_task_t() : task_base_t("imu", 0U, 1024U, pyro::task_base_t::priority_t::HIGH) {}

  protected:
    /* 器件初始化 + VQF 配置都放在 init()：task_base_t::start() 会在调用者上下文
     * 同步执行它，因此 imu_service_start() 返回时就能拿到真实结果。
     * 调用时机要求调度器已启动（read() 内部要等 DMA 完成信号量）。 */
    pyro::status_t init() override
    {
        pyro::dwt_drv_t::init(k_cpu_mhz); /* 用于统计 VQF 单批耗时 */

        const bool ok = s_imu.init();
        imu_service_init_result = ok ? 0 : 1;
        if (!ok)
        {
            return pyro::PYRO_ERROR;
        }

        const pyro::status_t ret = s_vqf.configure(k_vqf_cfg);
        imu_service_vqf_ready = (ret == pyro::PYRO_OK) ? 1 : 0;
        return ret;
    }

    void run_loop() override
    {
        uint32_t us_max = 0U;
        uint32_t next_1s = xTaskGetTickCount() + 1000U;
        TickType_t wake = xTaskGetTickCount();

        for (;;)
        {
            /* ---- ① 读取一拍：内部完成状态查询与两段 DMA 等待 ----
             * 本任务不感知数据来源（FIFO / 直读），也不感知状态字节。 */
            if (s_imu.read())
            {
                ++imu_service_read_ok;

                imu_data_t &data = s_imu.read_imu_data();
                const uint8_t ng = data.gyro.count;
                const uint8_t na = data.acc.count;

                /* 观测用：最新一帧原始物理量 */
                if (ng != 0U)
                {
                    const gyro_sample_t &g = data.gyro.cur();
                    imu_service_gyro_rps[0] = g.Gyro[0];
                    imu_service_gyro_rps[1] = g.Gyro[1];
                    imu_service_gyro_rps[2] = g.Gyro[2];
                }
                if (na != 0U)
                {
                    const acc_sample_t &a = data.acc.cur();
                    imu_service_accel_mps2[0] = a.Accel[0];
                    imu_service_accel_mps2[1] = a.Accel[1];
                    imu_service_accel_mps2[2] = a.Accel[2];
                }

                /* ---- ② 融合：单任务下环形缓冲在本段执行期间不会被写入，
                 *       head 稳定，at(i) 读到的一定是连续样本。 ---- */
                const uint32_t t0 = pyro::dwt_drv_t::get_current_ticks();
                const pyro::status_t st = s_vqf.update(data);
                const uint32_t us = (pyro::dwt_drv_t::get_current_ticks() - t0) / k_cpu_mhz;
                if (us > us_max) { us_max = us; }

                imu_service_vqf_status = static_cast<int32_t>(st);
                if (st == pyro::PYRO_WARNING)
                {
                    ++imu_service_vqf_warn; /* 单帧异常但已跳过，继续 */
                }
                else if (st != pyro::PYRO_OK)
                {
                    ++imu_service_vqf_err;  /* 参数非法，本批提前结束 */
                }

                const pyro::vqf_t::state_t state = s_vqf.get_state();

                /* ---- ③ 按 VQF 实际消费量归还 ----
                 * ERROR 时未消费的样本留在环里，下批继续；不会重复也不会丢。 */
                s_imu.assume_gyro_point(state.consumed_gyr);
                s_imu.assume_acc_point(state.consumed_acc);
                imu_service_vqf_consumed +=
                    static_cast<uint32_t>(state.consumed_gyr) + state.consumed_acc;
                ++imu_service_algo_rounds;

                /* ---- ④ 姿态快照：写全本地结构体后用 seqlock 发布 ---- */
                float q[4];
                float sq = 0.0f;
                for (uint8_t i = 0U; i < 4U; ++i)
                {
                    q[i] = state.quat[i];
                    sq += q[i] * q[i];
                }

                float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
                quat_to_euler(q, roll, pitch, yaw);

                imu_attitude_t next{};
                for (uint8_t i = 0U; i < 4U; ++i) { next.quat[i] = q[i]; }
                for (uint8_t i = 0U; i < 3U; ++i) { next.bias[i] = state.bias[i]; }
                next.roll   = roll;
                next.pitch  = pitch;
                next.yaw    = yaw;
                next.rest_t = state.rest_t;
                next.rest   = state.rest_detected;
                next.valid  = state.initialized;

                ++s_attitude_seq; /* 奇数：更新中 */
                s_attitude = next;
                ++s_attitude_seq; /* 偶数：稳定 */

                /* 调试观测镜像 */
                for (uint8_t i = 0U; i < 4U; ++i) { imu_service_quat[i] = q[i]; }
                for (uint8_t i = 0U; i < 3U; ++i) { imu_service_bias_rps[i] = state.bias[i]; }
                imu_service_quat_norm = std::sqrt(sq);
                imu_service_roll      = roll;
                imu_service_pitch     = pitch;
                imu_service_yaw       = yaw;
                imu_service_rest      = state.rest_detected ? 1U : 0U;
                imu_service_rest_t    = state.rest_t;
            }

            /* 驱动诊断计数（总线/数据/超时） */
            const lsm6dsv16x_drv_t::diag_t d = s_imu.diag();
            imu_service_err_bus     = d.err_bus;
            imu_service_err_data    = d.err_data;
            imu_service_err_timeout = d.err_timeout;

            /* 每秒窗口：刷新 VQF 单批耗时峰值 */
            if (static_cast<int32_t>(xTaskGetTickCount() - next_1s) >= 0)
            {
                next_1s += 1000U;
                imu_service_vqf_us_max = us_max;
                us_max = 0U;
            }

            /* 固定周期：把本拍耗时从间隔里扣掉，避免执行时间累积成降频。 */
            vTaskDelayUntil(&wake, 1U);
        }
    }
};

imu_task_t s_imu_task;
bool s_started = false;

} // namespace

/* ============================== 对外接口 ============================== */

imu_data_t &imu_service_data()
{
    return s_imu.read_imu_data();
}

imu_attitude_t imu_service_attitude()
{
    /* seqlock 读侧：seq 为偶数且拷贝前后未变，说明拿到的是一次完整更新。
     * 写方（采集任务）可能在拷贝中途抢占，此时 seq 变化，重试即可。
     * 重试上界很小：每次更新只占几十条指令。 */
    imu_attitude_t snapshot{};
    for (;;)
    {
        const uint32_t seq_before = s_attitude_seq;
        if ((seq_before & 1U) != 0U)
        {
            continue; /* 正在写，等下一个完整状态 */
        }
        snapshot = s_attitude;
        __DMB(); /* 保证结构体拷贝先于 seq 的再次读取完成 */
        if (s_attitude_seq == seq_before)
        {
            break;
        }
    }
    return snapshot;
}

pyro::status_t imu_service_start()
{
    if (s_started)
    {
        return pyro::PYRO_ALREADY_INIT;
    }

    /* 单任务结构：init() 里同步完成器件初始化与 VQF 配置，
     * 失败原因经 imu_service_init_result / imu_service_vqf_ready 外露。 */
    const pyro::status_t ret = s_imu_task.start();
    if (ret != pyro::PYRO_OK)
    {
        return ret;
    }

    s_started = true;
    imu_service_heap_free_min = static_cast<uint32_t>(xPortGetFreeHeapSize());
    return pyro::PYRO_OK;
}
