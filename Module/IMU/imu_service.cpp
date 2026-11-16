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
 * 代价：VQF 的耗时计入 1 ms 周期内。实测（1920 Hz，约 2 样本/路）：update() 平均 518 µs、
 * 峰值 793 µs；整拍（读+融合+发布）平均 668 µs，循环速率 ≈999 Hz。
 * 峰值距 1 ms 还有约 207 µs 留给两段 SPI/DMA 读取与快照发布；这一余量来自两轮优化
 * （矩阵暂存复用、块结构协方差传播，详见 PYRo/Algorithm/VQF/README.md）。
 * 若后续加入更多算法导致超标，应通过降低 ODR、降低观测频率或分频送入融合解决，
 * 而不是重新拆成双任务（双任务的坑见上）。耗时分布与优化记录见 README。
 *
 * ---------------------------- 关于实时性 ----------------------------
 * read() 内部经 DMA 完成信号量让出 CPU，不轮询；SPI 传输期间任务处于阻塞态。
 * VQF 计算期间本任务占满 CPU，但它是最低耦合的做法：1 ms 周期内做完即返回。
 */

#include "imu_service.h"

#include "imu_mount.h"
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
volatile uint32_t imu_service_vqf_us_avg   = 0U;   /* 每秒窗口内 update() 平均耗时 */
volatile uint32_t imu_service_loop_us_avg  = 0U;   /* 每秒窗口内整拍平均耗时（读+融合+发布） */
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
volatile uint32_t imu_service_heading_zeroed  = 0U;
volatile uint32_t imu_service_heading_quality = 0U;
volatile float    imu_service_acc_confidence = 0.0f;
volatile float    imu_service_acc_nis       = 0.0f;
volatile uint32_t imu_service_acc_rejected  = 0U;
}

namespace
{

constexpr uint32_t k_cpu_mhz = 170U;

/* ======================= 产品配置 =======================
 * 两路 ODR 为 1920 Hz。采集与姿态传播使用全部样本，ESKF 倾斜/静止观测约 500 Hz。
 * 单任务预算为 1 ms，imu_service_vqf_us_max 记录目标硬件的实际单批峰值。 */
const imu_cfg_t k_imu_cfg = {
    false,                  /* calibrate_flag ：关闭上电标定，零偏交给 VQF 静止估计 */
    500U,                   /* calibrate_times：1920 Hz 下约 260 ms */
    4U,                     /* acc_range      ：±4 g   */
    1920U,                  /* acc_freq       ：1920 Hz */
    2000U,                  /* gyro_range     ：±2000 dps */
    1920U,                  /* gyro_freq      ：1920 Hz */
    true,                   /* fifo_en        ：true=FIFO 批量；false=寄存器直读回退 */
};

/* ======================= VQF 配置 =======================
 * 门限按本板静止实测噪声设定：驱动只做单位换算与零偏扣除，输出为器件坐标系，
 * 静止时角速度残余约 1 °/s 量级（零偏未扣除），故 rest_th_gyr 放到 5 °/s。
 * 加速度静止波动约 0.2~0.3 m/s²，rest_th_acc 放到 1.0 留余量。
 * bias_clip 必须 ≥ rest_th_gyr，否则低通值被提前截断，静止永不成立。 */
const pyro::vqf_t::config_t k_vqf_cfg = [] {
    pyro::vqf_t::config_t c;
    c.tau_acc      = 0.4f;  /* 加速度倾斜校正时间常数，s（默认 3.0 偏慢） */
    c.rest_min_t   = 1.0f;   /* 静止确认时间，s */
    c.rest_th_gyr  = 5.0f;   /* °/s */
    c.rest_th_acc  = 1.0f;   /* m/s² */
    c.bias_clip    = 5.0f;   /* °/s，与 rest_th_gyr 匹配 */
    c.motion_bias_est_enabled = true;  /* 可信倾斜观测参与联合 ESKF 零偏估计 */
    c.rest_bias_est_enabled   = true;  /* 保留静止零偏估计 */
    c.acc_update_period = 0.002f;       /* 倾斜/静止观测约 500 Hz，陀螺按全部样本传播 */
    return c;
}();

/* ======================= 航向归零门控 =======================
 * 6 轴 VQF 的航向不可观测，yaw 零点必须由产品定义。建立零点需要三件事同时成立：
 *   ① state.initialized   —— 可信重力方向已建立，四元数才有意义；
 *   ② state.rest_detected —— 已含 rest_min_t 的连续静止确认（VQF 内部判定）；
 *   ③ bias_sigma 低于门限 —— 该量是零偏协方差三轴的上界，而竖直轴只在静止观测下
 *                            可观测，也正是产生 yaw 漂移的那一轴。未收敛就归零，
 *                            零点会带上截至该时刻的累积漂移，且后续静止观测会追溯
 *                            修正 yaw 误差，使发布 yaw 在收敛过程中继续移动。
 *                            （零偏收敛后漂移即停止，并非长期漂移。）
 * 再加超时兜底，避免产品在运动中被开机时永远给不出可信 yaw。
 * 注意量纲：state.bias_sigma 为 rad/s，而 config.bias_sigma_init 为 °/s。 */
constexpr float    k_bias_sigma_ok_rps  = 0.005f;  /* rad/s，≈0.29 °/s */
constexpr uint32_t k_zero_timeout_round = 5000U;   /* 5 s @1 kHz */

/* ======================= 板级绑定 ======================= */
/* 片选由产品装配层绑定（CubeMX 中 PA4 初始为高）。换芯片/换板只改这两行。 */
GPIO_TypeDef *const k_cs_port = GPIOA;
const uint16_t k_cs_pin = GPIO_PIN_4;

/* ======================= 总线与驱动装配 ======================= */
/* 换 SPI/I²C 只需换这一行；驱动不感知总线类型。
 * 第 4 参 dma_enable：true = 收发走 DMA，完成经中断通知并唤醒任务。 */
lsm6dsv16x_spi_t s_bus(&hspi1, k_cs_port, k_cs_pin, true);
lsm6dsv16x_drv_t s_imu(k_imu_cfg, s_bus);

/* VQF/ESKF 实例采用固定存储，静态区分配。 */
pyro::vqf_t s_vqf;

/* ======================= 装配归一化 =======================
 * 安装方向配置与映射表见 Module/Config/imu_mount.h。器件驱动只输出器件坐标系的
 * 物理量，映射在快照写入点完成（P3）。参数在初始化时算一次，热路径只做查表与乘法。 */
float s_m_inv[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /* m 的逆四元数（姿态换系右乘用） */

/* 最新解包物理量（器件坐标系），供调试镜像按装配系输出 */
float s_last_acc_dev[3]  = {0.0f, 0.0f, 0.0f};
float s_last_gyro_dev[3] = {0.0f, 0.0f, 0.0f};

/* 姿态快照：采集任务写、其他任务读。
 * 用版本号（seqlock）保证读方拿到的一定是完整的一次更新：
 *   写方：seq++（变奇数=更新中）→ 写结构体 → seq++（变偶数=稳定）
 *   读方：读 seq 为偶数 → 拷贝 → 再读 seq，未变则有效，否则重试 */
imu_attitude_t s_attitude{};
volatile uint32_t s_attitude_seq = 0U;

/* ZYX 偏航角（rad）。欧拉角导出与航向归零共用，保持唯一真源。 */
float quat_yaw_rad(const float q[4])
{
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

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

    yaw = quat_yaw_rad(q) * rad_to_deg;
}

/* ======================= 航向归零 =======================
 * 只把 yaw 归零，roll/pitch 不动（重力参考不丢）：
 *   ✗ q0⁻¹ ⊗ q       —— 会把整个姿态打平，产品失去"水平"概念
 *   ✓ Rz(-yaw0) ⊗ q  —— 左乘纯偏航，ZYX 约定下恰好只加在 yaw 上
 * 实现位置为纯输出端后处理，不修改 VQF 内部状态——内部 acc_reference 与
 * last_acc_lp 活在 gyr_quat 张成的近惯性系里，改内部姿态会破坏门控历史。
 * VQF 重建时间基准（reset / tick_s 改变）后世界系重建，零点必须一并作废。 */
struct heading_ref_t
{
    float   yaw0    = 0.0f;  /* 归零时刻的 yaw，rad */
    bool    zeroed  = false;
    uint8_t quality = 0U;
};

heading_ref_t s_heading{};
uint32_t      s_zero_wait = 0U;

/* 左乘 Rz(-yaw0)，只改 yaw 分量。 */
void apply_heading_zero(float q[4], float yaw0)
{
    const float h = 0.5f * yaw0;
    const float c = std::cos(h), s = std::sin(h);
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    q[0] = c * w + s * z;
    q[1] = c * x + s * y;
    q[2] = c * y - s * x;
    q[3] = c * z - s * w;
}

/* ======================= 采集 + 融合任务 ======================= */
class imu_task_t final : public pyro::task_base_t
{
  public:
    /* 栈 1024 词 = 4 KB，承载采集、ESKF 和快照发布。
     * 通过编译器栈用量报告检查调用链，板级验证时继续检查栈高水位。 */
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

        /* 装配归一化参数：由 imu_mount.h 的正向表导出，只算一次。
         * 换安装方向改 k_imu_direct 即可，驱动侧无需改动。 */
        mount_quat_inverse(k_imu_direct, s_m_inv);

        const pyro::status_t ret = s_vqf.configure(k_vqf_cfg);
        imu_service_vqf_ready = (ret == pyro::PYRO_OK) ? 1 : 0;
        return ret;
    }

    void run_loop() override
    {
        uint32_t us_max = 0U;
        uint32_t us_sum = 0U;      /* 窗口内 update() 累计耗时 */
        uint32_t win_rounds = 0U;  /* 窗口内实际执行融合的拍数 */
        uint32_t loop_sum = 0U;    /* 窗口内整拍累计耗时 */
        uint32_t loop_cnt = 0U;
        uint32_t next_1s = xTaskGetTickCount() + 1000U;
        TickType_t wake = xTaskGetTickCount();

        for (;;)
        {
            const uint32_t t_loop0 = pyro::dwt_drv_t::get_current_ticks();
            /* ---- ① 读取一拍：内部完成状态查询与两段 DMA 等待 ----
             * 本任务不感知数据来源（FIFO / 直读），也不感知状态字节。 */
            if (s_imu.read())
            {
                ++imu_service_read_ok;

                imu_data_t &data = s_imu.read_imu_data();
                const uint8_t ng = data.gyro.count;
                const uint8_t na = data.acc.count;

                /* 观测用：缓存最新一帧原始物理量（器件坐标系，装配归一化后再输出） */
                if (ng != 0U)
                {
                    const gyro_sample_t &g = data.gyro.cur();
                    s_last_gyro_dev[0] = g.Gyro[0];
                    s_last_gyro_dev[1] = g.Gyro[1];
                    s_last_gyro_dev[2] = g.Gyro[2];
                }
                if (na != 0U)
                {
                    const acc_sample_t &a = data.acc.cur();
                    s_last_acc_dev[0] = a.Accel[0];
                    s_last_acc_dev[1] = a.Accel[1];
                    s_last_acc_dev[2] = a.Accel[2];
                }

                /* ---- ② 融合：单任务下环形缓冲在本段执行期间不会被写入，
                 *       head 稳定，at(i) 读到的一定是连续样本。 ---- */
                const uint32_t t0 = pyro::dwt_drv_t::get_current_ticks();
                const pyro::status_t st = s_vqf.update(data);
                const uint32_t us = (pyro::dwt_drv_t::get_current_ticks() - t0) / k_cpu_mhz;
                if (us > us_max) { us_max = us; }
                us_sum += us;
                ++win_rounds;

                imu_service_vqf_status = static_cast<int32_t>(st);
                if (st == pyro::PYRO_WARNING)
                {
                    ++imu_service_vqf_warn; /* 单帧异常但已跳过，继续 */
                }
                else if (st != pyro::PYRO_OK)
                {
                    ++imu_service_vqf_err;  /* 参数非法，本批提前结束 */
                }

                pyro::vqf_t::state_t state;
                s_vqf.get_state(state);

                /* ---- ③ 按 VQF 实际消费量归还 ----
                 * ERROR 时未消费的样本留在环里，下批继续；不会重复也不会丢。 */
                s_imu.assume_gyro_point(state.consumed_gyr);
                s_imu.assume_acc_point(state.consumed_acc);
                imu_service_vqf_consumed +=
                    static_cast<uint32_t>(state.consumed_gyr) + state.consumed_acc;
                ++imu_service_algo_rounds;

                /* ---- ④ 姿态快照：写全本地结构体后用 seqlock 发布 ---- */

                /* ④-1 装配归一化：器件系 → 安装系。
                 * 换系在归零之前，因为航向零点是【安装系】下的产品定义。
                 * 机体系向量（含零偏）都要换，漏一个即与四元数不同系。 */
                float q[4];
                for (uint8_t i = 0U; i < 4U; ++i) { q[i] = state.quat[i]; }
                quat_to_install(q, s_m_inv);

                float gyro_inst[3] = {s_last_gyro_dev[0], s_last_gyro_dev[1], s_last_gyro_dev[2]};
                float acc_inst[3]  = {s_last_acc_dev[0],  s_last_acc_dev[1],  s_last_acc_dev[2]};
                float bias_inst[3] = {state.bias[0], state.bias[1], state.bias[2]};
                mount_apply(gyro_inst, k_imu_direct);
                mount_apply(acc_inst,  k_imu_direct);
                mount_apply(bias_inst, k_imu_direct);

                /* ④-2 建立航向零点（一次性）。yaw0 取自本拍，故归零那一拍输出 yaw 恰为 0。 */
                if (!s_heading.zeroed)
                {
                    const bool settled = state.initialized && state.rest_detected &&
                                         (state.bias_sigma < k_bias_sigma_ok_rps);
                    const bool fallback =
                        (++s_zero_wait >= k_zero_timeout_round) && state.initialized;
                    if (settled || fallback)
                    {
                        s_heading.yaw0    = quat_yaw_rad(q);
                        s_heading.zeroed  = true;
                        s_heading.quality = settled ? 1U : 2U;
                    }
                }
                if (s_heading.zeroed) { apply_heading_zero(q, s_heading.yaw0); }

                float sq = 0.0f;
                for (uint8_t i = 0U; i < 4U; ++i) { sq += q[i] * q[i]; }

                /* ④-3 欧拉角从【换系 + 归零之后】的四元数导出（欧拉角不是线性量） */
                float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
                quat_to_euler(q, roll, pitch, yaw);

                imu_attitude_t next{};
                for (uint8_t i = 0U; i < 4U; ++i) { next.quat[i] = q[i]; }
                for (uint8_t i = 0U; i < 3U; ++i) { next.bias[i] = bias_inst[i]; }
                next.roll   = roll;
                next.pitch  = pitch;
                next.yaw    = yaw;
                next.rest_t = state.rest_t;
                next.rest   = state.rest_detected;
                next.valid  = state.initialized;
                next.heading_zeroed  = s_heading.zeroed;
                next.heading_quality = s_heading.quality;

                /* 写侧在【挂起调度器】的极短窗口内完成 置奇 → 写入 → 置偶：
                 *   ① 若只靠两次自增，更高优先级的读者会读到奇序号后一直空转，
                 *      而写方正是被它抢占的——双方互等（活锁）；
                 *   ② vTaskSuspendAll 只禁止任务切换，不屏蔽中断，SPI/DMA 完成
                 *      照常进 ISR，与"用临界区屏蔽 DMA 中断导致采集降频"的老坑不同；
                 *   ③ 窗口内只有约 60 字节赋值与两次自增（≈1 µs），不进入慢路径。
                 * __DMB 保证结构体赋值不会被编译器挪到第二次自增之后。 */
                vTaskSuspendAll();
                ++s_attitude_seq; /* 奇数：更新中 */
                s_attitude = next;
                __DMB();
                ++s_attitude_seq; /* 偶数：稳定 */
                (void)xTaskResumeAll();

                /* ④-4 调试观测镜像：与快照同坐标系、同零点 */
                for (uint8_t i = 0U; i < 4U; ++i) { imu_service_quat[i] = q[i]; }
                for (uint8_t i = 0U; i < 3U; ++i) {
                    imu_service_bias_rps[i]   = bias_inst[i];
                    imu_service_gyro_rps[i]   = gyro_inst[i];
                    imu_service_accel_mps2[i] = acc_inst[i];
                }
                imu_service_quat_norm = std::sqrt(sq);
                imu_service_roll      = roll;
                imu_service_pitch     = pitch;
                imu_service_yaw       = yaw;
                imu_service_rest      = state.rest_detected ? 1U : 0U;
                imu_service_rest_t    = state.rest_t;
                imu_service_heading_zeroed  = s_heading.zeroed ? 1U : 0U;
                imu_service_heading_quality = s_heading.quality;
                imu_service_acc_confidence = state.acc_confidence;
                imu_service_acc_nis = state.acc_nis;
                imu_service_acc_rejected = state.acc_rejected ? 1U : 0U;
            }

            /* 驱动诊断计数（总线/数据/超时） */
            const lsm6dsv16x_drv_t::diag_t d = s_imu.diag();
            imu_service_err_bus     = d.err_bus;
            imu_service_err_data    = d.err_data;
            imu_service_err_timeout = d.err_timeout;

            /* 每秒窗口：刷新 VQF 单批耗时峰值与整拍/融合平均耗时 */
            loop_sum += (pyro::dwt_drv_t::get_current_ticks() - t_loop0) / k_cpu_mhz;
            ++loop_cnt;
            if (static_cast<int32_t>(xTaskGetTickCount() - next_1s) >= 0)
            {
                next_1s += 1000U;
                imu_service_vqf_us_max  = us_max;
                imu_service_vqf_us_avg  = (win_rounds > 0U) ? (us_sum / win_rounds) : 0U;
                imu_service_loop_us_avg = (loop_cnt > 0U) ? (loop_sum / loop_cnt) : 0U;
                us_max = 0U;
                us_sum = 0U;
                win_rounds = 0U;
                loop_sum = 0U;
                loop_cnt = 0U;
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

imu_attitude_t imu_service_attitude()
{
    /* seqlock 读侧：seq 为偶数且拷贝前后未变，说明拿到的是一次完整更新。
     * 读方可能在拷贝中途被写方抢占（写方优先级更高），此时 seq 变化，重试即可。
     *
     * 奇序号空转不会活锁：写方把"置奇→写入→置偶"包在 vTaskSuspendAll 窗口内，
     * 任务级读者不可能观察到奇序号。代价是【不能中断上下文调用】——ISR 若读到
     * 奇序号，写方永远不会有 CPU 把它置回偶数。 */
    imu_attitude_t snapshot{};
    for (;;)
    {
        const uint32_t seq_before = s_attitude_seq;
        if ((seq_before & 1U) != 0U)
        {
            continue; /* 保留为防御：写方窗口内被 ISR 读取时会走到这里 */
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
