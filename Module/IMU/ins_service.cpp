/**
 * @file    ins_service.cpp
 * @brief   INS 服务实现：采集 → 融合 → 装配归一化 → 航向归零 → 统一快照。
 *
 * ============================ 数据流 ============================
 *   ① imu_base_drv::read()   驱动内部完成"FIFO 状态 → 等 DMA 完成 → 读数据 → 解包"，
 *                            输出器件坐标系的最新物理量（单位换算 + 器件零偏已扣）
 *   ② pyro::vqf_t::update()  按样本自带时间戳合并两路，做姿态 + 零偏融合
 *   ③ 按算法报告的消费量归还环形缓冲（ERROR 时未消费样本留到下一拍）
 *   ④ 装配归一化：器件系 → 安装系（四元数右乘 m⁻¹；向量/零偏查表换轴换号）
 *   ⑤ 航向归零：Rz(-yaw0) 左乘，只改 yaw，roll/pitch 与重力参考不动
 *   ⑥ 欧拉角从【换系 + 归零之后】的四元数导出，随后 seqlock 发布快照
 *
 * 单任务与实时性的取舍见 ins_service.h 文件头。
 */

#include "ins_service.h"

#include <cmath>

#include "FreeRTOS.h"
#include "imu_config.h"
#include "imu_mount.h"
#include "lsm6dsv16x_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_task.h"
#include "spi.h"
#include "task.h"

/* ======================= 调试观测变量 =======================
 * 必须放在【全局作用域】：带 extern "C" 的定义如果留在命名空间里，调试器会把
 * 它记成 `imu_module::imu_service_xxx`，而 ELF 里是 C 链接的裸名字，
 * gdb 按名字查会得到 "Missing ELF symbol"。调试脚本与文档都直接按裸名字引用。 */
extern "C" {
volatile int32_t  imu_service_init_result    = 1;
volatile int32_t  imu_service_vqf_ready      = 0;
volatile uint32_t imu_service_read_ok        = 0U;
volatile uint32_t imu_service_algo_rounds    = 0U;
volatile uint32_t imu_service_vqf_consumed   = 0U;
volatile uint32_t imu_service_err_bus        = 0U;
volatile uint32_t imu_service_err_data       = 0U;
volatile uint32_t imu_service_err_ovr        = 0U;
volatile uint32_t imu_service_err_backpressure = 0U;
volatile uint32_t imu_service_err_timeout    = 0U;
volatile uint32_t imu_service_vqf_err        = 0U;
volatile uint32_t imu_service_vqf_warn       = 0U;
volatile int32_t  imu_service_vqf_status     = 0;
volatile uint32_t imu_service_vqf_us_max     = 0U;
volatile uint32_t imu_service_vqf_us_avg     = 0U;
volatile uint32_t imu_service_loop_us_avg    = 0U;
volatile uint32_t imu_service_heap_free_min  = 0U;
volatile uint32_t imu_service_ins_stack_free = 0U;
volatile uint32_t imu_service_cap_stack_free = 0U;
volatile uint32_t imu_service_cap_rounds     = 0U;
volatile uint32_t imu_service_cap_us_max     = 0U;
volatile uint32_t imu_service_cap_us_avg     = 0U;
volatile float    imu_service_quat[4]        = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float    imu_service_quat_norm      = 1.0f;
volatile float    imu_service_accel_mps2[3]  = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_gyro_rps[3]    = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_bias_rps[3]    = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_roll           = 0.0f;
volatile float    imu_service_pitch          = 0.0f;
volatile float    imu_service_yaw            = 0.0f;
volatile uint32_t imu_service_rest           = 0U;
volatile float    imu_service_rest_t         = 0.0f;
volatile uint32_t imu_service_heading_zeroed  = 0U;
volatile uint32_t imu_service_heading_quality = 0U;
volatile float    imu_service_acc_confidence  = 0.0f;
volatile float    imu_service_acc_nis         = 0.0f;
volatile uint32_t imu_service_acc_rejected    = 0U;
}

namespace imu_module
{

namespace
{

/* ======================= 板级绑定与总线装配 =======================
 * 硬件对象放文件作用域而不是类成员：类声明就能保持零 HAL 依赖，
 * Communication 层只需 include ins_service.h / imu_snapshot.h。
 * 换芯片/换板只改这三行（片选由产品装配层绑定，CubeMX 中 PA4 初始为高）。 */
GPIO_TypeDef *const k_cs_port = GPIOA;
constexpr uint16_t  k_cs_pin  = GPIO_PIN_4;

lsm6dsv16x_spi_t s_bus(&hspi1, k_cs_port, k_cs_pin, true);
lsm6dsv16x_drv_t s_imu(config::driver_config(), s_bus);
pyro::vqf_t      s_vqf;

/* 装配归一化参数：由 imu_mount.h 的正向表导出，初始化时算一次 */
float s_m_inv[4] = {1.0f, 0.0f, 0.0f, 0.0f};

/* 反向封包参数（Mᵀ / Sa / Sg / 静态零偏），随快照一起发布 */
imu_publish_params_t s_publish_params{};

/* 最新一帧解包物理量（器件坐标系）；装配归一化后写入快照 */
float s_last_acc_dev[3]  = {0.0f, 0.0f, 0.0f};
float s_last_gyro_dev[3] = {0.0f, 0.0f, 0.0f};

/* 1 秒窗口统计（采集 / 融合各一套，互不影响），只用于观测，不影响功能。
 * 【必须各用各的窗口边界】：两个任务曾共用 s_next_1s，采集先到就把边界推进，
 * 融合侧的窗口条件几乎永不成立，表现为 vqf_us_avg/ins_stack_free 恒为 0。 */
uint32_t s_us_max    = 0U;
uint32_t s_us_sum    = 0U;
uint32_t s_win_round = 0U;
uint32_t s_loop_sum  = 0U;
uint32_t s_loop_cnt  = 0U;
uint32_t s_algo_next_1s = 0U;   /* 融合任务的 1 s 窗口边界 */

/* 采集任务侧窗口 */
uint32_t s_cap_us_max = 0U;
uint32_t s_cap_us_sum = 0U;
uint32_t s_cap_win    = 0U;
uint32_t s_cap_next_1s = 0U;    /* 采集任务的 1 s 窗口边界 */

/* ZYX 偏航角（rad）。欧拉角导出与航向归零共用，保证唯一真源。 */
float quat_yaw_rad(const float q[4])
{
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

/* 四元数 → 欧拉角（ZYX 顺序，°）。只用于对外快照与调试观测。 */
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

/* 航向归零：左乘 Rz(-yaw0)，只改 yaw 分量。
 *   ✗ q0⁻¹ ⊗ q       —— 会把整个姿态打平，产品失去"水平"概念
 *   ✓ Rz(-yaw0) ⊗ q  —— 左乘纯偏航，ZYX 约定下恰好只加在 yaw 上
 * 纯输出端后处理，不修改算法内部状态（内部近惯性系历史不能被改）。 */
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

} // namespace

/* ======================= 常驻任务 =======================
 * 采集与融合分成两个任务（2026-11-16，SPSC 落地之后）：
 *   ins_cap  —— 只做 read()，1 ms 硬周期，优先级 HIGH；周期不受融合耗时影响
 *   ins_algo —— 只做 update() + 快照发布，优先级 ABOVE_NORMAL，尽力而为
 *
 * 交接完全依赖 SPSC 环形缓冲：采集只写 tail（publish），算法只写 head（release），
 * 因此【全程不需要任何临界区】。旧单任务版把两者串行，融合耗时计入 1 ms 预算；
 * 旧双任务版（SPSC 之前）用临界区保护，反而屏蔽了 SPI/DMA 中断。
 *
 * 器件初始化与算法配置放在采集任务的 init() 里（task_base_t::start() 在调用者
 * 上下文执行它），保证 imu_service_t::init() 返回时已拿到真实结果。 */
class cap_task_t final : public pyro::task_base_t
{
  public:
    cap_task_t()
        : task_base_t("ins_cap", 0U, config::kCapTaskStackWords,
                      pyro::task_base_t::priority_t::HIGH)
    {
    }

  protected:
    pyro::status_t init() override;
    void run_loop() override
    {
        /* 采集节拍是产品的硬约束：read() 内部的两段 DMA 等待会挂起任务，
         * 因此本任务绝大多数时间处于阻塞态，融合再慢也不会挤占这个周期。 */
        TickType_t wake = xTaskGetTickCount();
        for (;;)
        {
            ins_service_t::instance()._capture_loop();
            vTaskDelayUntil(&wake, config::kPublishPeriodMs);
        }
    }
};

class ins_task_t final : public pyro::task_base_t
{
  public:
    ins_task_t()
        : task_base_t("ins_algo", 0U, config::kInsTaskStackWords,
                      pyro::task_base_t::priority_t::ABOVE_NORMAL)
    {
    }

  protected:
    pyro::status_t init() override { return pyro::PYRO_OK; }
    void run_loop() override
    {
        /* 尽力而为：每拍尽量算一次。融合耗时超过 1 ms 时本任务会自然降频，
         * 但采集周期不受影响——这正是拆任务的目的。 */
        TickType_t wake = xTaskGetTickCount();
        for (;;)
        {
            ins_service_t::instance()._round();
            vTaskDelayUntil(&wake, config::kPublishPeriodMs);
        }
    }
};

cap_task_t s_cap_task;
ins_task_t s_algo_task;

/* 采集任务初始化：器件、总线、算法配置、装配参数都收在这里。 */
pyro::status_t cap_task_t::init()
{
    pyro::dwt_drv_t::init(config::kCpuMhz); /* 统计融合/采集单批耗时 */

    const bool ok = s_imu.init();
    imu_service_init_result = ok ? 0 : 1;
    if (!ok)
    {
        return pyro::PYRO_ERROR;
    }

    /* 装配归一化：正向换系四元数的逆（姿态右乘用） */
    mount_quat_inverse(k_imu_direct, s_m_inv);

    /* 反向封包参数：由同一张正向表导出逆表，Sa/Sg/b 直接取驱动的解包参数，
     * 保证通信层的反向换算与驱动正向换算同源。 */
    mount_inverse_indices(k_imu_direct, s_publish_params.inv_idx,
                          s_publish_params.inv_neg);
    s_publish_params.acc_mps2_per_lsb = s_imu.acc_mps2_per_lsb();
    s_publish_params.gyro_rps_per_lsb = s_imu.gyro_rps_per_lsb();
    const imu_cali_t &cali = s_imu.read_imu_data().cali;
    s_publish_params.gyro_bias_dev[0] = cali.gx_offset;
    s_publish_params.gyro_bias_dev[1] = cali.gy_offset;
    s_publish_params.gyro_bias_dev[2] = cali.gz_offset;

    const pyro::status_t cfg = s_vqf.configure(config::vqf_config());
    imu_service_vqf_ready = (cfg == pyro::PYRO_OK) ? 1 : 0;
    if (cfg != pyro::PYRO_OK)
    {
        return cfg;
    }

    s_cap_next_1s = static_cast<uint32_t>(xTaskGetTickCount()) + 1000U;
    s_algo_next_1s = s_cap_next_1s;
    return pyro::PYRO_OK;
}

/* ======================= 服务接口 ======================= */

ins_service_t &ins_service_t::instance()
{
    static ins_service_t service;
    return service;
}

pyro::status_t ins_service_t::init()
{
    if (_started)
    {
        return pyro::PYRO_ALREADY_INIT;
    }
    /* 顺序：先采集（完成器件初始化与算法配置），再算法（只消费缓冲）。 */
    const pyro::status_t cap = s_cap_task.start();
    if (cap != pyro::PYRO_OK)
    {
        return cap;
    }
    const pyro::status_t algo = s_algo_task.start();
    if (algo != pyro::PYRO_OK)
    {
        return algo;
    }
    _started = true;
    imu_service_heap_free_min = static_cast<uint32_t>(xPortGetFreeHeapSize());
    return pyro::PYRO_OK;
}

imu_snapshot_t ins_service_t::snapshot() const
{
    /* seqlock 读侧：序号为偶数且拷贝前后未变，说明拿到的是一次完整更新。
     * 奇序号空转不会活锁——写方把"置奇→写入→置偶"包在 vTaskSuspendAll 窗口内，
     * 任务级读者不可能观察到奇序号；代价是不可在 ISR 里调用（见头文件）。 */
    imu_snapshot_t copy{};
    for (;;)
    {
        const uint32_t seq_before = _snapshot_seq;
        if ((seq_before & 1U) != 0U)
        {
            continue; /* 防御：写方窗口内被 ISR 读取时会走到这里 */
        }
        copy = _snapshot;
        __DMB(); /* 保证结构体拷贝先于序号的再次读取完成 */
        if (_snapshot_seq == seq_before)
        {
            break;
        }
    }
    return copy;
}

void ins_service_t::_update_heading_zero(float yaw_install, bool initialized,
                                         bool rest, float bias_sigma_rps)
{
    if (_heading.zeroed)
    {
        return;
    }
    /* 三项条件同时成立：可信重力方向 + 连续静止 + 零偏收敛。
     * bias_sigma 是零偏协方差三轴的上界；竖直轴只在静止观测下可观测，也正是
     * 产生 yaw 漂移的那一轴。未收敛就归零，零点会带上累积漂移。
     * 再加超时兜底，避免运动开机时永远给不出可信 yaw。 */
    const bool settled = initialized && rest &&
                         (bias_sigma_rps < config::kHeadingBiasSigmaRps);
    const bool fallback = (++_zero_wait >= config::kHeadingTimeoutRounds) && initialized;
    if (settled || fallback)
    {
        _heading.yaw0    = yaw_install;
        _heading.zeroed  = true;
        _heading.quality = settled ? 1U : 2U;
    }
}

/* ======================= 采集任务体 =======================
 * 只做 read()：FIFO 状态 → 等 DMA → 读数据 → TAG 解包 → publish 到 SPSC 环形缓冲。
 * 不碰算法状态、不发布快照，因此本函数耗时与融合复杂度无关。
 * s_last_*_dev 是"最新一帧解包物理量"，由本任务写、算法任务读；
 * 它是 3 个 float 的独立快照量，允许陈旧一拍（协议取"最新驱动物理量"语义）。 */
void ins_service_t::_capture_loop()
{
    const uint32_t t0 = pyro::dwt_drv_t::get_current_ticks();
    if (s_imu.read())
    {
        ++imu_service_read_ok;

        imu_data_t &data = s_imu.read_imu_data();
        if (data.gyro.available() != 0U)
        {
            const gyro_sample_t &g = data.gyro.cur();
            s_last_gyro_dev[0] = g.Gyro[0];
            s_last_gyro_dev[1] = g.Gyro[1];
            s_last_gyro_dev[2] = g.Gyro[2];
        }
        if (data.acc.available() != 0U)
        {
            const acc_sample_t &a = data.acc.cur();
            s_last_acc_dev[0] = a.Accel[0];
            s_last_acc_dev[1] = a.Accel[1];
            s_last_acc_dev[2] = a.Accel[2];
        }
    }

    /* 驱动侧诊断直通（累计计数）：read() 失败时也要刷新，否则看不出坏在哪一步。
     * 数据源是驱动自己的计数器，本层不重复统计——镜像只是给调试器一个稳定入口。 */
    const lsm6dsv16x_drv_t::diag_t diag = s_imu.diag();
    imu_service_err_bus     = diag.err_bus;
    imu_service_err_data    = diag.err_data;
    imu_service_err_timeout = diag.err_timeout;
    imu_service_err_ovr     = diag.err_ovr;
    imu_service_err_backpressure = diag.err_backpressure;

    /* 采集侧耗时统计（含两段 DMA 等待，是"采集是否守住 1 ms"的直接口径）。 */
    const uint32_t us =
        static_cast<uint32_t>((pyro::dwt_drv_t::get_current_ticks() - t0) /
                              config::kCpuMhz);
    if (us > s_cap_us_max) { s_cap_us_max = us; }
    s_cap_us_sum += us;
    ++s_cap_win;                 /* 先计数，重置时才不会把本次漏掉 */
    ++imu_service_cap_rounds;

    /* 1 秒窗口：采集/融合各刷各的，互不影响。 */
    const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount());
    if (static_cast<int32_t>(now - s_cap_next_1s) >= 0)
    {
        s_cap_next_1s += 1000U;
        imu_service_cap_us_max = s_cap_us_max;
        imu_service_cap_us_avg = (s_cap_win > 0U) ? (s_cap_us_sum / s_cap_win) : 0U;
        imu_service_cap_stack_free = uxTaskGetStackHighWaterMark(nullptr);
        s_cap_us_max = 0U;
        s_cap_us_sum = 0U;
        s_cap_win    = 0U;
    }
}

void ins_service_t::_round()
{
    const uint32_t t_loop0 = pyro::dwt_drv_t::get_current_ticks();

    /* ---- 融合：本任务【只消费】环形缓冲。
     *      SPSC 保证生产者（采集任务）只移动 tail、本任务只移动 head，
     *      因此 at(i) 读到的一定是完整样本，且无需任何临界区。 ---- */
    {
        imu_data_t &data = s_imu.read_imu_data();
        const uint32_t t0 = pyro::dwt_drv_t::get_current_ticks();
        const pyro::status_t st = s_vqf.update(data);
        const uint32_t us =
            static_cast<uint32_t>((pyro::dwt_drv_t::get_current_ticks() - t0) /
                                  config::kCpuMhz);
        if (us > s_us_max) { s_us_max = us; }
        s_us_sum += us;
        ++s_win_round;

        imu_service_vqf_status = static_cast<int32_t>(st);
        if (st == pyro::PYRO_WARNING)
        {
            ++imu_service_vqf_warn; /* 单帧异常但已跳过，继续 */
        }
        else if (st != pyro::PYRO_OK)
        {
            ++imu_service_vqf_err; /* 参数非法，本批提前结束 */
        }

        pyro::vqf_t::state_t state;
        s_vqf.get_state(state);

        /* ---- 按算法实际消费量归还环形缓冲：
         *      ERROR 时未消费的样本留在环里，下一拍继续，不重复也不丢。 ---- */
        s_imu.assume_gyro_point(state.consumed_gyr);
        s_imu.assume_acc_point(state.consumed_acc);
        imu_service_vqf_consumed +=
            static_cast<uint32_t>(state.consumed_gyr) + state.consumed_acc;
        ++imu_service_algo_rounds;

        /* ---- 装配归一化：器件系 → 安装系。
         *      换系必须在归零之前，因为航向零点是安装系下的产品定义；
         *      机体系向量（含零偏）都要换，漏一个即与四元数不同系。 ---- */
        float q[4] = {state.quat[0], state.quat[1], state.quat[2], state.quat[3]};
        quat_to_install(q, s_m_inv);

        float acc_inst[3]  = {s_last_acc_dev[0], s_last_acc_dev[1], s_last_acc_dev[2]};
        float gyro_inst[3] = {s_last_gyro_dev[0], s_last_gyro_dev[1], s_last_gyro_dev[2]};
        float bias_inst[3] = {state.bias[0], state.bias[1], state.bias[2]};
        mount_apply(acc_inst, k_imu_direct);
        mount_apply(gyro_inst, k_imu_direct);
        mount_apply(bias_inst, k_imu_direct);

        /* ---- 航向零点：yaw0 取自本拍，因此归零那一拍的输出 yaw 恰为 0 ---- */
        _update_heading_zero(quat_yaw_rad(q), state.initialized, state.rest_detected,
                             state.bias_sigma);
        if (_heading.zeroed)
        {
            apply_heading_zero(q, _heading.yaw0);
        }

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        quat_to_euler(q, roll, pitch, yaw);

        /* ---- 组快照（全量 POD，seqlock 发布） ---- */
        imu_snapshot_t next{};
        next.sequence     = _sequence++;
        next.timestamp_ms = static_cast<uint32_t>(xTaskGetTickCount());
        for (uint8_t i = 0U; i < 4U; ++i) { next.quat[i] = q[i]; }
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            next.accel[i] = acc_inst[i];
            next.gyro[i]  = gyro_inst[i];
            next.bias[i]  = bias_inst[i];
        }
        next.euler[0] = roll;
        next.euler[1] = pitch;
        next.euler[2] = yaw;
        next.rest_t   = state.rest_t;
        next.rest     = state.rest_detected;
        next.valid    = state.initialized;
        next.heading_zeroed  = _heading.zeroed;
        next.heading_quality = _heading.quality;
        next.params   = s_publish_params;

        /* 写侧挂起调度器：更高优先级的读者若读到奇序号会一直空转，
         * 而写方正是被它抢占的——双方互等。vTaskSuspendAll 只禁止任务切换、
         * 不屏蔽中断，SPI/DMA 完成照常进 ISR。窗口内只有约 100 字节赋值。 */
        vTaskSuspendAll();
        ++_snapshot_seq; /* 奇数：更新中 */
        _snapshot = next;
        __DMB();
        ++_snapshot_seq; /* 偶数：稳定 */
        (void)xTaskResumeAll();

        /* ---- 调试镜像 ---- */
        float sq = 0.0f;
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            imu_service_quat[i] = q[i];
            sq += q[i] * q[i];
        }
        imu_service_quat_norm = std::sqrt(sq);
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            imu_service_accel_mps2[i] = acc_inst[i];
            imu_service_gyro_rps[i]   = gyro_inst[i];
            imu_service_bias_rps[i]   = bias_inst[i];
        }
        imu_service_roll  = roll;
        imu_service_pitch = pitch;
        imu_service_yaw   = yaw;
        imu_service_rest  = state.rest_detected ? 1U : 0U;
        imu_service_rest_t        = state.rest_t;
        imu_service_heading_zeroed  = _heading.zeroed ? 1U : 0U;
        imu_service_heading_quality = _heading.quality;
        imu_service_acc_confidence  = state.acc_confidence;
        imu_service_acc_nis         = state.acc_nis;
        imu_service_acc_rejected    = state.acc_rejected ? 1U : 0U;
    }

    /* ---- 1 秒窗口统计：本任务的融合单批与整拍耗时（采集侧另有一套） ---- */
    s_loop_sum += static_cast<uint32_t>(
        (pyro::dwt_drv_t::get_current_ticks() - t_loop0) / config::kCpuMhz);
    ++s_loop_cnt;
    if (static_cast<int32_t>(static_cast<uint32_t>(xTaskGetTickCount()) - s_algo_next_1s) >= 0)
    {
        s_algo_next_1s += 1000U;
        imu_service_vqf_us_max  = s_us_max;
        imu_service_vqf_us_avg  = (s_win_round > 0U) ? (s_us_sum / s_win_round) : 0U;
        imu_service_loop_us_avg = (s_loop_cnt > 0U) ? (s_loop_sum / s_loop_cnt) : 0U;
        /* 栈余量（词）：随窗口一起刷新，用于确认 768 词是否够用 */
        imu_service_ins_stack_free = uxTaskGetStackHighWaterMark(nullptr);
        s_us_max    = 0U;
        s_us_sum    = 0U;
        s_win_round = 0U;
        s_loop_sum  = 0U;
        s_loop_cnt  = 0U;
    }
}

} // namespace imu_module
