/**
 * @file    imu_fusion.cpp
 * @brief   融合侧实现：VQF/ESKF + 装配归一化 + 航向归零 + 统一快照。
 *
 * ============================ 职责边界 ============================
 * 本模块只消费采集侧的 SPSC 环形缓冲，产出【安装坐标系、航向已归零】的
 * 单一快照；不碰总线、不创建任务。
 *
 * 数据流：
 *   capture::data()（器件系）
 *     └→ vqf.update()            姿态 + 零偏（器件系）
 *          └→ 装配归一化          q⊗m⁻¹ + 向量查表（器件系 → 安装系）
 *               └→ 航向归零        Rz(-yaw0)⊗q（只改 yaw）
 *                    └→ 欧拉角 ZYX（从归零后的四元数导出）
 *                         └→ seqlock 发布快照
 */

#include "imu_fusion.h"

#include <cmath>

#include "FreeRTOS.h"
#include "imu_capture.h"
#include "imu_config.h"
#include "imu_mount.h"
#include "pyro_algo_vqf_eskf.h"
#include "pyro_dwt_drv.h"
#include "task.h"

/* ======================= 调试观测变量 =======================
 * 与采集侧一样放全局作用域：extern "C" 定义若留在命名空间里，
 * 调试器记的名字与 ELF 裸名字对不上，gdb 查不到。 */
extern "C" {
volatile int32_t  imu_service_vqf_ready       = 0;
volatile uint32_t imu_service_algo_rounds     = 0U;
volatile uint32_t imu_service_vqf_consumed    = 0U;
volatile uint32_t imu_service_vqf_err         = 0U;
volatile uint32_t imu_service_vqf_warn        = 0U;
volatile uint32_t imu_service_vqf_resets      = 0U;
volatile uint32_t imu_service_reset_reason    = 0U;
volatile uint32_t imu_service_invalid_frames  = 0U;
volatile uint32_t imu_service_snapshot_valid  = 0U;
volatile int32_t  imu_service_vqf_status      = 0;
volatile uint32_t imu_service_vqf_us_max      = 0U;
volatile uint32_t imu_service_vqf_us_avg      = 0U;
volatile uint32_t imu_service_loop_us_avg     = 0U;
volatile uint32_t imu_service_ins_stack_free  = 0U;
volatile float    imu_service_quat[4]         = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float    imu_service_quat_norm       = 1.0f;
volatile float    imu_service_accel_mps2[3]   = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_gyro_rps[3]     = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_bias_rps[3]     = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_roll            = 0.0f;
volatile float    imu_service_pitch           = 0.0f;
volatile float    imu_service_yaw             = 0.0f;
volatile uint32_t imu_service_rest            = 0U;
volatile float    imu_service_rest_t          = 0.0f;
volatile float    imu_service_rest_gyr_lp[3]  = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_rest_acc_lp[3]  = {0.0f, 0.0f, 0.0f};
volatile uint32_t imu_service_heading_zeroed  = 0U;
volatile uint32_t imu_service_heading_quality = 0U;
volatile float    imu_service_acc_confidence  = 0.0f;
volatile float    imu_service_acc_nis         = 0.0f;
volatile uint32_t imu_service_acc_rejected    = 0U;
}

namespace imu_module
{
namespace fusion
{
namespace
{

pyro::vqf_eskf_t s_vqf;

/* 装配归一化参数：由 imu_mount.h 的正向表导出，初始化时算一次 */
float s_m_inv[4] = {1.0f, 0.0f, 0.0f, 0.0f};

/* 反向封包参数（Mᵀ / Sa / Sg / 静态零偏），随快照一起发布 */
imu_publish_params_t s_publish_params{};

/* 融合侧 1 s 窗口统计（与采集侧各用各的边界，互不影响）。 */
uint32_t s_us_max       = 0U;
uint32_t s_us_sum       = 0U;
uint32_t s_win_round    = 0U;
uint32_t s_loop_sum     = 0U;
uint32_t s_loop_cnt     = 0U;
uint32_t s_algo_next_1s = 0U;

/* 航向零点：yaw0 与质量位。6 轴 yaw 不可观测，零点必须由产品定义。 */
struct heading_ref_t
{
    float   yaw0    = 0.0f;
    bool    zeroed  = false;
    uint8_t quality = 0U;    /**< 0 未建立 / 1 静止收敛 / 2 超时兜底 */
};
heading_ref_t s_heading{};
TickType_t s_heading_started = 0U;

template <typename Sample>
struct latest_sample_t
{
    Sample sample{};
    TickType_t processed_at = 0U;
    bool seen = false;

    bool expired(TickType_t now) const
    {
        return seen && (now - processed_at >= pdMS_TO_TICKS(config::kSampleStaleMs));
    }
};

latest_sample_t<acc_sample_t> s_acc;
latest_sample_t<gyro_sample_t> s_gyro;

/* 重置原因码。显式赋值而不依赖枚举自增：调试器里看 imu_service_reset_reason
 * 需要能直接对照到原因，且后续增删原因时不应让既有取值漂移。
 *
 * 原有一项 capture_loss（软件层丢样代次变化）已移除：出现样本流断裂必然意味着
 * 别处代码有误或硬件故障，前者应在调试阶段解决、后者运行期无法修复；
 * 而且每丢一两个样本就重置滤波器，代价（valid=false 期间不输出、航向零点
 * 重建）远大于收益。真正的长时间断裂由 VQF 自身的 max_sample_gap_s 捕获。 */
enum class reset_reason_t : uint32_t
{
    stale        = 1U,   /* 超过 max_sample_gap_s 没有新样本（生产端停了） */
    rollback     = 2U,   /* 时间戳倒退，该帧已隔离 */
    gap          = 3U,   /* 时间戳跨度过大（VQF err_stage=6） */
    filter_error = 4U    /* 其他滤波器错误 */
};

uint32_t s_sequence = 0U;

/* 快照发布（seqlock）：写方在 vTaskSuspendAll 窗口内完成"置奇→写→置偶"。 */
imu_snapshot_t    s_snapshot{};
volatile uint32_t s_snapshot_seq = 0U;

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

/* 航向零点门控：三项条件同时成立才建立，另加超时兜底。
 * bias_sigma 是零偏协方差三轴的上界；竖直轴只在静止观测下可观测，
 * 也正是产生 yaw 漂移的那一轴。未收敛就归零，零点会带上累积漂移。 */
void update_heading_zero(float yaw_install, bool initialized, bool rest,
                         float bias_sigma_rps)
{
    if (s_heading.zeroed)
    {
        return;
    }
    const bool settled = initialized && rest &&
                         (bias_sigma_rps < config::kHeadingBiasSigmaRps);
    const bool fallback = initialized &&
        (xTaskGetTickCount() - s_heading_started >= pdMS_TO_TICKS(config::kHeadingTimeoutMs));
    if (settled || fallback)
    {
        s_heading.yaw0    = yaw_install;
        s_heading.zeroed  = true;
        s_heading.quality = settled ? 1U : 2U;
    }
}

} // namespace

pyro::status_t init()
{
    /* 装配归一化：正向换系四元数的逆（姿态右乘用） */
    mount_quat_inverse(k_imu_direct, s_m_inv);

    /* 反向封包参数：由同一张正向表导出逆表，Sa/Sg/b 直接取采集侧的解包参数，
     * 保证通信层的反向换算与驱动正向换算同源。 */
    mount_inverse_indices(k_imu_direct, s_publish_params.inv_idx,
                          s_publish_params.inv_neg);
    /* 编码系数由协议层按模块量程自行定义（协议自包含），
     * 这里只导出装配方向表——它是产品几何属性，与器件无关。 */

    const pyro::status_t cfg = s_vqf.configure(config::vqf_config());
    imu_service_vqf_ready = (cfg == pyro::PYRO_OK) ? 1 : 0;
    if (cfg != pyro::PYRO_OK)
    {
        return cfg;
    }

    s_algo_next_1s = static_cast<uint32_t>(xTaskGetTickCount()) + 1000U;
    s_heading_started = xTaskGetTickCount();
    return pyro::PYRO_OK;
}

namespace
{
void reset_fusion(pyro::vqf_eskf_t::state_t &state, reset_reason_t reason)
{
    ++imu_service_vqf_resets;
    imu_service_reset_reason = static_cast<uint32_t>(reason);
    s_acc.seen = false;
    s_gyro.seen = false;
    s_heading = {};
    s_heading_started = xTaskGetTickCount();
    pyro::status_t result = s_vqf.reset(state.bias, state.bias_sigma);
    if (result != pyro::PYRO_OK)
    {
        result = s_vqf.reset();
    }
    imu_service_vqf_ready = (result == pyro::PYRO_OK) ? 1 : 0;
    if (result != pyro::PYRO_OK)
    {
        imu_service_vqf_status = static_cast<int32_t>(result);
        ++imu_service_vqf_err;
    }
    s_vqf.get_state(state);
    state.initialized = false;
}

void publish_snapshot(const pyro::vqf_eskf_t::state_t &state)
{
    /* 装配归一化：器件系 → 安装系。
     * 换系必须在归零之前，因为航向零点是安装系下的产品定义；
     * 机体系向量（含零偏）都要换，漏一个即与四元数不同系。 */
    float q[4] = {state.quat[0], state.quat[1], state.quat[2], state.quat[3]};
    quat_to_install(q, s_m_inv);

    const float *last_acc  = s_acc.sample.Accel;
    const float *last_gyro = s_gyro.sample.Gyro;
    float acc_inst[3]  = {last_acc[0], last_acc[1], last_acc[2]};
    /* 角速度先扣掉 VQF 在线估计的零偏，再换到安装系对外发布。
     * 顺序不能反：_state.bias 与 last_gyro 同为机体系，必须先在同一坐标系里
     * 相减，再一起旋转；若先换系，零偏就落在了错误的坐标系上。
     * 这样对外发出的是"算法处理后的最终角速度"，接收端无需再做零偏补偿。 */
    float gyro_inst[3] = {last_gyro[0] - state.bias[0],
                          last_gyro[1] - state.bias[1],
                          last_gyro[2] - state.bias[2]};
    float bias_inst[3] = {state.bias[0], state.bias[1], state.bias[2]};
    mount_apply(acc_inst, k_imu_direct);
    mount_apply(gyro_inst, k_imu_direct);
    mount_apply(bias_inst, k_imu_direct);

    /* 航向零点：yaw0 取自本拍，因此归零那一拍的输出 yaw 恰为 0 */
    update_heading_zero(quat_yaw_rad(q), state.initialized, state.rest_detected,
                        state.bias_sigma);
    if (s_heading.zeroed)
    {
        apply_heading_zero(q, s_heading.yaw0);
    }

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    quat_to_euler(q, roll, pitch, yaw);

    /* ---- 组快照（全量 POD，seqlock 发布） ---- */
    imu_snapshot_t next{};
    next.sequence     = s_sequence++;
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
    next.heading_zeroed  = s_heading.zeroed;
    next.heading_quality = s_heading.quality;
    next.params   = s_publish_params;

    /* 写侧挂起调度器：更高优先级的读者若读到奇序号会一直空转，
     * 而写方正是被它抢占的——双方互等。vTaskSuspendAll 只禁止任务切换、
     * 不屏蔽中断，SPI/DMA 完成照常进 ISR。窗口内只有约 100 字节赋值。 */
    vTaskSuspendAll();
    ++s_snapshot_seq; /* 奇数：更新中 */
    s_snapshot = next;
    __DMB();
    ++s_snapshot_seq; /* 偶数：稳定 */
    (void)xTaskResumeAll();

    imu_service_snapshot_valid = next.valid ? 1U : 0U;
    if (!next.valid)
    {
        ++imu_service_invalid_frames;
    }

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
    imu_service_rest_t = state.rest_t;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        imu_service_rest_gyr_lp[i] = state.rest_gyr_lp[i];
        imu_service_rest_acc_lp[i] = state.rest_acc_lp[i];
    }
    imu_service_heading_zeroed  = s_heading.zeroed ? 1U : 0U;
    imu_service_heading_quality = s_heading.quality;
    imu_service_acc_confidence  = state.acc_confidence;
    imu_service_acc_nis         = state.acc_nis;
    imu_service_acc_rejected    = state.acc_rejected ? 1U : 0U;
}

bool process_batch()
{
    imu_data_t batch{};
    // 固定两路 tail 和丢失代次。释放 head 前生产者不能覆盖本批的槽。
    vTaskSuspendAll();
    const imu_data_t &live = capture::data();
    batch.gyro = live.gyro;
    batch.acc = live.acc;
    batch.timestamp_tick_s = live.timestamp_tick_s;
    (void)xTaskResumeAll();

    const uint16_t gyro_count = batch.gyro.available();
    const uint16_t acc_count = batch.acc.available();
    const TickType_t now = xTaskGetTickCount();
    const bool stale = s_gyro.expired(now) || s_acc.expired(now);
    if (!stale && (gyro_count == 0U && acc_count == 0U))
    {
        return false;
    }

    pyro::vqf_eskf_t::state_t state;
    if (stale)
    {
        /* 只在"根本没有新样本"时重置。样本流中途丢几个不重置：
         * VQF 按时间戳 dt 积分，丢样只表现为 dt 略大；真正的长时间断裂
         * 由它自己的 max_sample_gap_s 检测（err_stage=6）兜住。 */
        capture::release_consumed(gyro_count, acc_count);
        s_vqf.get_state(state);
        reset_fusion(state, reset_reason_t::stale);
    }
    else if (imu_service_vqf_ready)
    {
        const uint32_t t0 = pyro::dwt_drv_t::get_current_ticks();
        const pyro::status_t result = s_vqf.update(batch);
        const uint32_t us = (pyro::dwt_drv_t::get_current_ticks() - t0) / config::kCpuMhz;
        if (us > s_us_max) { s_us_max = us; }
        s_us_sum += us;
        ++s_win_round;
        imu_service_vqf_status = static_cast<int32_t>(result);
        s_vqf.get_state(state);
        imu_service_vqf_consumed += static_cast<uint32_t>(state.consumed_gyr) + state.consumed_acc;
        if (result == pyro::PYRO_OK || result == pyro::PYRO_WARNING)
        {
            if (result == pyro::PYRO_WARNING) { ++imu_service_vqf_warn; }
            if (state.updated_gyr != 0U)
            {
                s_gyro = {batch.gyro.at(state.updated_gyr - 1U), now, true};
            }
            if (state.updated_acc != 0U)
            {
                s_acc = {batch.acc.at(state.updated_acc - 1U), now, true};
            }
            capture::release_consumed(state.consumed_gyr, state.consumed_acc);
            if (s_gyro.seen && s_acc.seen && (state.updated_gyr != 0U || state.updated_acc != 0U))
            {
                publish_snapshot(state);
                return true;
            }
            return false;
        }

        ++imu_service_vqf_err;
        const uint32_t stage = pyro::vqf_eskf_t::err_stage;
        if (stage == 4U || stage == 6U)
        {
            // 倒退帧已隔离。长间隔后的首帧留给新时间基准。
            capture::release_consumed(state.consumed_gyr, state.consumed_acc);
            reset_fusion(state, stage == 4U ? reset_reason_t::rollback : reset_reason_t::gap);
        }
        else
        {
            capture::release_consumed(gyro_count, acc_count);
            reset_fusion(state, reset_reason_t::filter_error);
        }
    }
    else
    {
        capture::release_consumed(gyro_count, acc_count);
        return false;
    }

    // 有效 → 无效时发送一次零四元数，随后等待真实的新样本。
    if (s_snapshot.valid)
    {
        publish_snapshot(state);
        return true;
    }
    return false;
}
} // namespace

bool fuse_once()
{
    const uint32_t t_loop0 = pyro::dwt_drv_t::get_current_ticks();
    const bool fused = process_batch();
    ++imu_service_algo_rounds;

    /* 1 秒窗口统计：融合单批与整拍耗时（采集侧另有一套） */
    s_loop_sum += static_cast<uint32_t>(
        (pyro::dwt_drv_t::get_current_ticks() - t_loop0) / config::kCpuMhz);
    ++s_loop_cnt;
    if (static_cast<int32_t>(static_cast<uint32_t>(xTaskGetTickCount()) - s_algo_next_1s) >= 0)
    {
        s_algo_next_1s += 1000U;
        imu_service_vqf_us_max     = s_us_max;
        imu_service_vqf_us_avg     = (s_win_round > 0U) ? (s_us_sum / s_win_round) : 0U;
        imu_service_loop_us_avg    = (s_loop_cnt > 0U) ? (s_loop_sum / s_loop_cnt) : 0U;
        imu_service_ins_stack_free = uxTaskGetStackHighWaterMark(nullptr);
        s_us_max    = 0U;
        s_us_sum    = 0U;
        s_win_round = 0U;
        s_loop_sum  = 0U;
        s_loop_cnt  = 0U;
    }

    return fused;
}

imu_snapshot_t snapshot()
{
    /* seqlock 读侧：序号为偶数且拷贝前后未变，说明拿到的是一次完整更新。
     * 奇序号空转不会活锁——写方把"置奇→写入→置偶"包在 vTaskSuspendAll 窗口内，
     * 任务级读者不可能观察到奇序号；代价是不可在 ISR 里调用。 */
    imu_snapshot_t copy{};
    for (;;)
    {
        const uint32_t seq_before = s_snapshot_seq;
        if ((seq_before & 1U) != 0U)
        {
            continue; /* 防御：写方窗口内被 ISR 读取时会走到这里 */
        }
        copy = s_snapshot;
        __DMB(); /* 保证结构体拷贝先于序号的再次读取完成 */
        if (s_snapshot_seq == seq_before)
        {
            break;
        }
    }
    return copy;
}

} // namespace fusion
} // namespace imu_module
