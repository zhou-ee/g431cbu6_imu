// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
/**
 * @file pyro_algo_vqf.h
 * @brief 六轴 VQF 近惯性系预处理与全局姿态误差 ESKF。
 * VQF preprocessing with a global attitude-error / gyro-bias ESKF.
 * Adapted from VQF by Daniel Laidig and Thomas Seel.
 */
#ifndef PYRO_ALGO_VQF_H
#define PYRO_ALGO_VQF_H

#include "pyro_algo_kf.h"
#include "pyro_algo_butterworth.h"

struct imu_data_t;

namespace pyro
{

/** 配置中的时间使用 s，角度使用 °，加速度使用 m/s²。 */
struct vqf_config_t
{
    float tau_acc = 3.0f; ///< 近惯性系二阶加速度低通时间常数。
    bool motion_bias_est_enabled = true; ///< 可信运动观测同时修正三轴零偏。
    bool rest_bias_est_enabled = true; ///< 静止检测及静止零偏观测。
    float bias_sigma_init = 0.5f; ///< 初始零偏标准差，°/s。
    float bias_forgetting_time = 100.0f; ///< 零偏随机游走增长到 0.1 °/s 的时间。
    float bias_clip = 2.0f; ///< 三轴零偏限幅及陀螺静止门限，°/s。
    float bias_sigma_rest = 0.03f; ///< 静止角速度观测标准差，°/s。
    float rest_min_t = 1.0f; ///< 静止确认时间。
    float rest_filter_tau = 0.5f; ///< 静止检测参考低通时间常数。
    float rest_th_gyr = 2.0f; ///< 角速度偏差阈值，°/s。
    float rest_th_acc = 0.5f; ///< 加速度偏差阈值，m/s²。

    float attitude_sigma_init = 30.0f; ///< 初始姿态误差标准差，°。
    float gyro_noise_density = 0.05f; ///< 陀螺白噪声密度，°/s/√Hz。
    float acc_noise = 0.3f; ///< 单轴加速度噪声标准差，m/s²。
    float acc_norm_tolerance = 1.0f; ///< 模长偏差降权尺度，m/s²。
    float acc_vibration_tolerance = 1.0f; ///< 近惯性系高频残差 RMS 降权尺度，m/s²。
    float acc_update_period = 0.002f; ///< ESKF 观测最小间隔，陀螺按全部时间戳传播。
    float acc_recovery_t = 0.2f; ///< 启动及受扰后，可信加速度的连续确认时间。
    float acc_nis_threshold = 9.21034f; ///< 两维倾斜新息门限，默认 χ²(2) 的 99% 分位数。
};

/**
 * 固定存储的项目定制 VQF/ESKF，同一实例由调用方同步。
 * 角速度输入 rad/s，加速度输入 m/s²，输出 q 为传感器系到 Z-up 系的 [w,x,y,z]。
 * 陀螺积分、低通与协方差均使用样本的真实时间戳。
 */
class vqf_t
{
  public:
    using config_t = vqf_config_t;
    using vector3_t = vector_t<3>;
    using quaternion_t = vector_t<4>;

    struct state_t
    {
        quaternion_t quat{1.0f, 0.0f, 0.0f, 0.0f};
        vector3_t bias{}; ///< rad/s。
        matrix_t<3, 3> bias_covariance{}; ///< (rad/s)²。
        matrix_t<6, 6> error_covariance{}; ///< 顺序 [全局 δθ(rad), 机体系 δb(rad/s)]。
        float bias_sigma = 0.0f; ///< 零偏标准差上界，rad/s。
        bool rest_detected = false;
        bool initialized = false; ///< 已完成可信重力方向初始化。
        float rest_t = 0.0f;
        vector3_t rest_gyr_lp{};
        vector3_t rest_acc_lp{};
        uint8_t consumed_gyr = 0;
        uint8_t consumed_acc = 0;
        float acc_confidence = 0.0f; ///< 最近加速度的可信度，0..1。
        float acc_nis = 0.0f; ///< 最近一次两维倾斜观测的 NIS。
        bool acc_rejected = false; ///< 最近样本受到门控或处于恢复等待。
        uint32_t acc_corrections = 0; ///< 成功提交的 ESKF 倾斜观测数。
        uint32_t acc_rejections = 0; ///< 被拒绝或处于恢复等待的加速度帧数。
        kf_t<6, 3, true>::diagnostics_t bias_diagnostics{}; ///< 联合误差滤波器诊断，保留字段名。
    };

    /** 首次调用初始化，后续调用保留姿态/零偏并应用参数。
     * 非法输入保留配置和状态。改变低通/门控参数时重新确认可信加速度。
     * 初始标准差参数在 reset 时应用。 */
    status_t configure(const config_t &config = config_t{});
    /** 重置时间基准、姿态、协方差与所有滤波历史。bias/sigma 使用 rad/s。
     * sigma=0 使用 bias_sigma_init，重置后重新确认重力方向。 */
    status_t reset(const vector3_t &bias = vector3_t{}, float sigma = 0.0f);
    /** 按时间戳合并两路未消费样本，同时间戳先陀螺后加速度。
     * 各路各自单调并共享 tick_s，批次时间跨度 < 2^31 tick，支持 uint32 回绕。
     * 首帧建立时钟，重复时间戳消费后返回 WARNING，倒退帧保留并报 PARAM_ERROR。
     * 设备计数器重置或 tick_s 改变前调用 reset。
     * consumed_gyr/acc 为本次实际消费前缀，调用方按计数归还。
     * 门控/恢复等待返回 WARNING 并继续本批。其他错误终止批次，保留已提交步骤。
     * 加速度帧在处理开始后计入消费量，避免重试时重复推进滤波历史。 */
    status_t update(const imu_data_t &data);
    [[nodiscard]] state_t get_state() const;

    // 诊断分支，0 正常，1 流非法，2 tick_s 非法，3 tick_s 改变，4 时间倒退，5 dt 非法。
    inline static volatile uint32_t err_stage = 0U;

  private:
    using error_kf_t = kf_t<6, 3, true>;
    struct sample_clock_t
    {
        uint32_t timestamp = 0;
        bool initialized = false;
    };
    struct filter_state_t
    {
        quaternion_t gyr_quat{1.0f, 0.0f, 0.0f, 0.0f};
        quaternion_t acc_quat{1.0f, 0.0f, 0.0f, 0.0f};
        vector3_t bias{};
        vector3_t last_acc_lp{};
        vector3_t acc_reference{};
        vector3_t rest_gyr_lp{};
        vector3_t rest_acc_lp{};
        float vibration_variance = 0.0f;
        float trusted_acc_t = 0.0f;
        float observation_t = 0.0f;
        float rest_t = 0.0f;
        float acc_confidence = 0.0f;
        float acc_nis = 0.0f;
        bool reference_initialized = false;
        bool tilt_initialized = false;
        bool rest_gyr_ok = false;
        bool rest_detected = false;
        bool acc_rejected = false;
        uint32_t acc_corrections = 0;
        uint32_t acc_rejections = 0;
        uint8_t consumed_gyr = 0;
        uint8_t consumed_acc = 0;
        sample_clock_t gyr_clock{};
        sample_clock_t acc_clock{};
        float timestamp_tick_s = 0.0f;
    };

    status_t _update_gyr(const vector3_t &gyr, float dt);
    status_t _update_acc(const vector3_t &acc, float dt);
    status_t _initialize_tilt();
    status_t _correct_acc(float interval);
    status_t _correct_rest(float interval);
    status_t _inject_error();
    status_t _gate(error_kf_t::correction_t &correction, bool acceleration);
    status_t _correct(const vector3_t &residual, const error_kf_t::observation_t &H,
                      const error_kf_t::measurement_noise_t &R, bool acceleration);
    void _reject_acc();

    config_t _config{};
    filter_state_t _state{};
    error_kf_t _error_kf{};
    error_kf_t::diagnostics_t _diagnostics{};
    butterworth_lpf_t<3> _acc_lp{};
    butterworth_lpf_t<3> _rest_gyr_lp{};
    butterworth_lpf_t<3> _rest_acc_lp{};
};

} // namespace pyro
#endif
