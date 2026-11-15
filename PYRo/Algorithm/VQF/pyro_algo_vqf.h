// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
/**
 * @file pyro_algo_vqf.h
 * @brief 六轴 VQF，复用 PYRo KF/Matrix 与 CMSIS 四元数运算。
 *        Six-axis VQF using PYRo KF/Matrix and CMSIS quaternion operations.
 * Adapted from the VQF algorithm by Daniel Laidig and Thomas Seel.
 */
#ifndef PYRO_ALGO_VQF_H
#define PYRO_ALGO_VQF_H

#include "pyro_algo_kf.h"
#include "pyro_algo_butterworth.h"

struct imu_data_t;

namespace pyro
{

/** @brief 六轴配置；时间用秒，角速度参数用 °/s。
 *         Six-axis configuration; times in seconds, rate parameters in deg/s.
 */
struct vqf_config_t
{
    float tau_acc = 3.0f; ///< 加速度低通时间常数 / Acceleration LP time constant.
    // 运动零偏估计涉及额外多通道低通与矩阵运算，默认关闭。
    bool motion_bias_est_enabled = false; ///< 运动零偏估计 / Motion bias estimation.
    bool rest_bias_est_enabled = true; ///< 静止检测及零偏估计 / Rest estimation.
    float bias_sigma_init = 0.5f; ///< 初始零偏标准差，°/s / Initial bias sigma.
    float bias_forgetting_time = 100.0f; ///< 不确定度增长时间，s / Forgetting time.
    float bias_clip = 2.0f; ///< 零偏/残差限幅，°/s / Bias and residual limit.
    float bias_sigma_motion = 0.1f; ///< 运动收敛标准差，°/s / Motion target sigma.
    float bias_vertical_forgetting_factor = 0.0001f; ///< 竖直弱约束权重 / Vertical weight.
    float bias_sigma_rest = 0.03f; ///< 静止收敛标准差，°/s / Rest target sigma.
    float rest_min_t = 1.0f; ///< 静止确认时间，s / Minimum rest duration.
    float rest_filter_tau = 0.5f; ///< 静止参考低通时间常数，s / Rest LP tau.
    float rest_th_gyr = 2.0f; ///< 角速度偏差阈值，°/s / Gyro deviation threshold.
    float rest_th_acc = 0.5f; ///< 加速度偏差阈值，m/s² / Acc deviation threshold.
};

/**
 * @brief 固定存储、多实例六轴姿态估计；同一实例由调用方同步。
 *        Fixed-storage multi-instance estimator; caller synchronizes each instance.
 * 输入 gyr 为 rad/s，acc 为 m/s²。内部按时间戳合并两路样本。
 * Inputs are rad/s and m/s²; streams are merged internally by timestamp.
 */
class vqf_t
{
  public:
    using config_t = vqf_config_t;
    using vector3_t = vector_t<3>;
    using quaternion_t = vector_t<4>;

    /** @brief 一次读取的结果快照 / Result snapshot returned by value. */
    struct state_t
    {
        quaternion_t quat{1.0f, 0.0f, 0.0f, 0.0f}; ///< [w,x,y,z]，传感器系到 Z-up 参考系.
        vector3_t bias{}; ///< 陀螺零偏，rad/s / Gyro bias.
        matrix_t<3, 3> bias_covariance{}; ///< (rad/s)².
        float bias_sigma = 0.0f; ///< 标准差上界，rad/s / Sigma bound.
        bool rest_detected = false;
        bool initialized = false;
        float rest_t = 0.0f; ///< 连续静止时长，s.
        vector3_t rest_gyr_lp{}; ///< 静止参考角速度，rad/s.
        vector3_t rest_acc_lp{}; ///< 静止参考加速度，m/s².
        uint8_t consumed_gyr = 0; ///< 最近一次 update 已消费的陀螺帧数 / Consumed gyro frames.
        uint8_t consumed_acc = 0; ///< 最近一次 update 已消费的加速度帧数 / Consumed acc frames.
        kf_t<3, 3>::diagnostics_t bias_diagnostics{};
    };

    /** @brief 首次调用初始化；运行中统一更新参数，失败保留配置和状态。
     *         Initialize on first call; reconfigure in place; invalid input retains state.
     * 保留姿态和零偏，降低 bias_clip 时立即限幅。tau 变化适配低通状态；
     * 静止判据变化重新计时。reset 使用 bias_sigma_init 设置先验。
     * Preserves attitude/bias, clamps to a reduced limit, adapts LP tau changes.
     * Rest criteria changes restart detection.
     * bias_sigma_init sets the prior on reset.
     */
    status_t configure(const config_t &config = config_t{});    /** @brief 重置全部状态，可指定初始零偏及标准差，单位 rad/s。
     *         Reset all state with optional initial bias/sigma in rad/s.
     * sigma=0 使用配置的 bias_sigma_init / Zero sigma uses configured prior.
     */
    status_t reset(const vector3_t &bias = vector3_t{}, float sigma = 0.0f);
    /** @brief 消费 imu_data_t 两路未消费的新样本，按时间戳合并。
     *         Consume both streams' pending samples, merged by timestamp.
     * 两路各自按采样时间排列，使用同一计数器；同时间戳先 gyr 后 acc。
     * Each stream is chronological on the same clock; gyro wins timestamp ties.
     * uint32 回绕比较要求本批时间跨度小于 2^31 tick。
     * Wraparound ordering assumes a batch spans less than 2^31 ticks.
     * 各路 dt = uint32_t(本帧 timestamp - 上帧 timestamp) * data.timestamp_tick_s。
     * Each stream derives elapsed seconds from its own timestamps and the clock tick.
     * 各路首帧建立时间基准并初始化低通，陀螺积分与静止计时从第二帧开始。
     * First samples prime filters and clocks; integration/timing starts on the next sample.
     * 同一路重复时间戳消费并返回 WARNING；倒退或间隔 >= 2^31 tick 返回 PARAM_ERROR。
     * Duplicate stream timestamps are consumed with WARNING; backward/ambiguous deltas fail.
     * 设备计数器重置后须 reset；数据和帧数由调用方管理。
     * Reset after a device clock reset; caller owns buffers and counts.
     * timestamp_tick_s 在 reset 前须保持一致。
     * Clock tick units must remain constant until reset.
     * WARNING 继续处理，其他错误结束批次，此前完成的步骤保留。
     * Warnings continue; errors stop the batch, preserving completed steps.
     * 根据快照 consumed_gyr/consumed_acc 归还样本；非法输入帧保留。
     * Return consumed prefixes using snapshot counts; invalid input frames remain.
     * 加速度校正已提交而 KF 失败时，当前加速度帧计入消费量，避免重复校正。
     * An acc frame with committed inclination correction counts even if KF fails.
     */
    status_t update(const imu_data_t &data);
    [[nodiscard]] state_t get_state() const;

  private:
    using bias_kf_t = kf_t<3, 3>;
    struct sample_clock_t
    {
        uint32_t timestamp = 0;
        bool initialized = false;
    };
    struct filter_state_t
    {
        quaternion_t gyr_quat{1.0f, 0.0f, 0.0f, 0.0f};
        quaternion_t acc_quat{1.0f, 0.0f, 0.0f, 0.0f};
        vector3_t last_acc_lp{};
        vector3_t rest_gyr_lp{};
        vector3_t rest_acc_lp{};
        float rest_t = 0.0f;
        bool rest_gyr_ok = false;
        bool rest_detected = false;
        uint8_t consumed_gyr = 0;
        uint8_t consumed_acc = 0;
        sample_clock_t gyr_clock{};
        sample_clock_t acc_clock{};
        float timestamp_tick_s = 0.0f;
    };
    struct coefficients_t
    {
        float bias_p0 = 0.0f;
        float bias_variance_rate = 0.0f;
        float bias_motion_p = 0.0f;
        float bias_rest_p = 0.0f;
    };
    struct bias_noise_t
    {
        float variance = 0.0f;
        float motion = 0.0f;
        float vertical = 0.0f;
        float rest = 0.0f;
    };

    status_t _prepare_bias(float dt);
    status_t _update_gyr(const vector3_t &gyr, float dt);
    status_t _update_acc(const vector3_t &acc, float dt);
    status_t _update_bias(const vector3_t &acc_earth, float dt);

    config_t _config{};
    filter_state_t _state{};
    coefficients_t _coeffs{};
    bias_noise_t _bias_noise{};
    float _bias_dt = 0.0f;
    bias_kf_t _bias_kf{};
    butterworth_lpf_t<3> _acc_lp{};
    butterworth_lpf_t<3> _rest_gyr_lp{};
    butterworth_lpf_t<3> _rest_acc_lp{};
    butterworth_lpf_t<9> _motion_rotation_lp{};
    butterworth_lpf_t<2> _motion_bias_lp{};

  public:
    /* 诊断：最近一次 update 走到哪个 PARAM_ERROR 分支（0 = 未报错）。
     *   1=stream 非法 2=tick_s 非法 3=tick_s 变化
     *   4=时间戳倒退 5=dt 非正
     * 仅用于上板定位，不影响算法行为。 */
    inline static volatile uint32_t err_stage = 0U;
};

} // namespace pyro
#endif
