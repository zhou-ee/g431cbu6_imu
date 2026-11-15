// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
#include "pyro_algo_vqf.h"
#include "imu_base_drv.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace pyro
{
namespace
{
constexpr float deg_to_rad = 0.01745329251994329577f;

float square(float value) { return value * value; }

float norm(const vector_t<3> &value)
{
    // sqrtf（float 版）：避免链接 double 版 __ieee754_sqrt
    return sqrtf(square(value[0]) + square(value[1]) + square(value[2]));
}

vqf_t::quaternion_t multiply(const vqf_t::quaternion_t &a,
                              const vqf_t::quaternion_t &b)
{
    vqf_t::quaternion_t result;
    arm_quaternion_product_single_f32(a.data(), b.data(), result.data());
    return result;
}

vqf_t::quaternion_t normalized(const vqf_t::quaternion_t &q)
{
    vqf_t::quaternion_t result;
    arm_quaternion_normalize_f32(q.data(), result.data(), 1);
    return result;
}

matrix_t<3, 3> rotation(const vqf_t::quaternion_t &q)
{
    matrix_t<3, 3> result;
    arm_quaternion2rotation_f32(q.data(), result.data(), 1);
    return result;
}

// v' = q * v * q⁻¹，直接旋转向量以省去中间旋转矩阵。
vector_t<3> rotate(const vqf_t::quaternion_t &q, const vector_t<3> &v)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    // t = 2 * (q_vec × v)
    const float tx = 2.0f * (y * v[2] - z * v[1]);
    const float ty = 2.0f * (z * v[0] - x * v[2]);
    const float tz = 2.0f * (x * v[1] - y * v[0]);
    // v' = v + w * t + q_vec × t
    return {v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx)};
}

matrix_t<3, 3> identity(float scale = 1.0f)
{
    return {scale, 0.0f, 0.0f, 0.0f, scale, 0.0f, 0.0f, 0.0f, scale};
}

template <typename Sample> bool valid_stream(const sample_stream_t<Sample> &stream)
{
    return stream.count == 0 ||
        (stream.sample && stream.count <= stream.capacity && stream.head < stream.capacity);
}
} // namespace

status_t vqf_t::configure(const config_t &config)
{
    config_t next = config;
    const float positive[] = {
        next.tau_acc, next.bias_sigma_init,
        next.bias_forgetting_time, next.bias_clip, next.bias_sigma_motion,
        next.bias_sigma_rest, next.rest_filter_tau, next.rest_th_gyr, next.rest_th_acc};
    for (float value : positive)
        if (!std::isfinite(value) || value <= 0.0f)
            return PYRO_PARAM_ERROR;
    if (!std::isfinite(next.rest_min_t) || next.rest_min_t < 0.0f ||
        !std::isfinite(next.bias_vertical_forgetting_factor) ||
        next.bias_vertical_forgetting_factor < 0.0f)
        return PYRO_PARAM_ERROR;

    // 协方差统一为 (rad/s)²，统一缩放保持原版 Kalman 增益。
    // SI covariance units preserve the original gain under common scaling.
    // 全 float 计算：只为求平方，用 x*x 代替 std::pow(x,2)，
    // 避免把 double 版 __ieee754_pow（约 3 KB）拉进固件。
    const float radians = deg_to_rad;
    const float variance_rate = (0.1f * radians) * (0.1f * radians) /
                           next.bias_forgetting_time;
    const float motion_p = (next.bias_sigma_motion * radians) *
                           (next.bias_sigma_motion * radians);
    const float rest_p = (next.bias_sigma_rest * radians) *
                         (next.bias_sigma_rest * radians);
    coefficients_t coeffs;
    coeffs.bias_p0 = square(next.bias_sigma_init * deg_to_rad);
    coeffs.bias_variance_rate = variance_rate;
    coeffs.bias_motion_p = motion_p;
    coeffs.bias_rest_p = rest_p;
    const float derived[] = {
        coeffs.bias_p0, coeffs.bias_variance_rate,
        coeffs.bias_motion_p, coeffs.bias_rest_p,
        next.bias_clip * deg_to_rad, next.rest_th_gyr * deg_to_rad};
    for (float value : derived)
        if (!std::isfinite(value) || value <= 0.0f)
            return PYRO_PARAM_ERROR;

    const bool first = !_bias_kf.is_initialized();
    if (first)
    {
        const status_t result = _bias_kf.init({}, identity(coeffs.bias_p0));
        if (result != PYRO_OK)
            return result;
    }
    const bool motion_changed = next.motion_bias_est_enabled != _config.motion_bias_est_enabled;
    const bool rest_changed = next.rest_bias_est_enabled != _config.rest_bias_est_enabled ||
        next.rest_filter_tau != _config.rest_filter_tau ||
        next.rest_min_t != _config.rest_min_t ||
        next.rest_th_gyr != _config.rest_th_gyr || next.rest_th_acc != _config.rest_th_acc ||
        next.bias_clip != _config.bias_clip;

    _acc_lp.configure(next.tau_acc);
    _rest_gyr_lp.configure(next.rest_filter_tau);
    _rest_acc_lp.configure(next.rest_filter_tau);
    _motion_rotation_lp.configure(next.tau_acc);
    _motion_bias_lp.configure(next.tau_acc);
    if (motion_changed)
    {
        _motion_rotation_lp.reset();
        _motion_bias_lp.reset();
    }
    if (rest_changed)
    {
        _rest_gyr_lp.reset();
        _rest_acc_lp.reset();
        _state.rest_gyr_lp = {};
        _state.rest_acc_lp = {};
        _state.rest_t = 0.0f;
        _state.rest_gyr_ok = false;
        _state.rest_detected = false;
    }
    _config = next;
    _coeffs = coeffs;
    _bias_dt = 0.0f;
    if (first)
        return reset();

    vector3_t bias = _bias_kf.get_state();
    const float clip = _config.bias_clip * deg_to_rad;
    for (float &value : bias)
        value = std::clamp(value, -clip, clip);
    if (bias != _bias_kf.get_state())
        return _bias_kf.reset(bias, _bias_kf.get_covariance());
    return PYRO_OK;
}

status_t vqf_t::reset(const vector3_t &bias, float sigma)
{
    if (!_bias_kf.is_initialized())
        return PYRO_NOT_FOUND;
    if (!matrix::is_finite(bias) || !std::isfinite(sigma) || sigma < 0.0f ||
        !std::isfinite(square(sigma)))
        return PYRO_PARAM_ERROR;
    const float clip = _config.bias_clip * deg_to_rad;
    for (float value : bias)
        if (std::fabs(value) > clip)
            return PYRO_PARAM_ERROR;
    _state = {};
    _bias_dt = 0.0f;
    _acc_lp.reset();
    _rest_gyr_lp.reset();
    _rest_acc_lp.reset();
    _motion_rotation_lp.reset();
    _motion_bias_lp.reset();
    return _bias_kf.reset(bias, identity(sigma > 0.0f ? square(sigma) : _coeffs.bias_p0));
}

status_t vqf_t::update(const imu_data_t &data)
{
    _state.consumed_gyr = 0;
    _state.consumed_acc = 0;
    if (!_bias_kf.is_initialized())
        return PYRO_NOT_FOUND;
    if (!valid_stream(data.gyro) || !valid_stream(data.acc))
        return PYRO_PARAM_ERROR;
    if (data.gyro.count == 0 && data.acc.count == 0)
        return PYRO_OK;
    if (!std::isfinite(data.timestamp_tick_s) || data.timestamp_tick_s <= 0.0f)
        return PYRO_PARAM_ERROR;
    if (_state.timestamp_tick_s != 0.0f && _state.timestamp_tick_s != data.timestamp_tick_s)
        return PYRO_PARAM_ERROR;

    uint8_t &gyr_index = _state.consumed_gyr;
    uint8_t &acc_index = _state.consumed_acc;
    status_t batch_result = PYRO_OK;
    while (gyr_index < data.gyro.count || acc_index < data.acc.count)
    {
        // 先检查有效帧，再短路比较时间戳；差值为零时优先陀螺仪。
        // Check bounds before timestamp reads; zero delta selects the gyro.
        const bool gyro_first = gyr_index < data.gyro.count &&
            (acc_index >= data.acc.count ||
             uint32_t(data.acc.at(acc_index).timestamp -
                      data.gyro.at(gyr_index).timestamp) < (uint32_t{1} << 31));
        const uint32_t timestamp = gyro_first
            ? data.gyro.at(gyr_index).timestamp
            : data.acc.at(acc_index).timestamp;
        sample_clock_t &clock = gyro_first ? _state.gyr_clock : _state.acc_clock;
        const uint32_t ticks = timestamp - clock.timestamp;
        if (clock.initialized && ticks >= (uint32_t{1} << 31))
            return PYRO_PARAM_ERROR;
        const float dt = clock.initialized ? static_cast<float>(ticks) * data.timestamp_tick_s : 0.0f;
        if (!std::isfinite(dt) || (clock.initialized && ticks > 0 && dt <= 0.0f))
            return PYRO_PARAM_ERROR;
        const bool duplicate = clock.initialized && ticks == 0;
        status_t result = PYRO_WARNING;
        if (gyro_first)
        {
            const auto &gyr = data.gyro.at(gyr_index).Gyro;
            if (duplicate && (!std::isfinite(gyr[0]) || !std::isfinite(gyr[1]) || !std::isfinite(gyr[2])))
                return PYRO_PARAM_ERROR;
            if (!duplicate)
                result = _update_gyr({gyr[0], gyr[1], gyr[2]}, dt);
            if (result != PYRO_OK && result != PYRO_WARNING)
                return result;
            ++gyr_index;
        }
        else
        {
            const auto &sample = data.acc.at(acc_index).Accel;
            const vector3_t acc{sample[0], sample[1], sample[2]};
            if (!std::isfinite(norm(acc)))
                return PYRO_PARAM_ERROR;
            if (!duplicate)
            {
                if (dt > 0.0f && (_config.motion_bias_est_enabled || _config.rest_bias_est_enabled))
                {
                    result = _prepare_bias(dt);
                    if (result != PYRO_OK)
                        return result;
                }
                result = _update_acc(acc, dt);
            }
            ++acc_index;
        }
        clock = {timestamp, true};
        _state.timestamp_tick_s = data.timestamp_tick_s;
        if (result == PYRO_WARNING)
            batch_result = PYRO_WARNING;
        else if (result != PYRO_OK)
            return result;
    }
    return batch_result;
}

status_t vqf_t::_update_gyr(const vector3_t &gyr, float dt)
{
    const vector3_t unbiased = gyr - _bias_kf.get_state();
    const float rate = norm(unbiased);
    if (!std::isfinite(rate) ||
        !std::isfinite(rate * dt))
        return PYRO_PARAM_ERROR;

    if (_config.rest_bias_est_enabled)
    {
        _rest_gyr_lp.process(gyr.data(), _state.rest_gyr_lp.data(), dt);
        const vector3_t deviation = gyr - _state.rest_gyr_lp;
        const float squared_deviation = square(deviation[0]) +
            square(deviation[1]) + square(deviation[2]);
        const float clip = _config.bias_clip * deg_to_rad;
        _state.rest_gyr_ok = squared_deviation < square(_config.rest_th_gyr * deg_to_rad) &&
            std::fabs(_state.rest_gyr_lp[0]) <= clip &&
            std::fabs(_state.rest_gyr_lp[1]) <= clip &&
            std::fabs(_state.rest_gyr_lp[2]) <= clip;
        if (!_state.rest_gyr_ok)
        {
            _state.rest_t = 0.0f;
            _state.rest_detected = false;
        }
    }

    if (dt > 0.0f && rate > std::numeric_limits<float>::epsilon())
    {
        const float half_angle = 0.5f * rate * dt;
        const float scale = std::sin(half_angle) / rate;
        const quaternion_t step{std::cos(half_angle), scale * unbiased[0],
                                 scale * unbiased[1], scale * unbiased[2]};
        _state.gyr_quat = normalized(multiply(_state.gyr_quat, step));
    }
    return PYRO_OK;
}

status_t vqf_t::_update_acc(const vector3_t &acc, float dt)
{
    if (acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f)
    {
        _state.rest_t = 0.0f;
        _state.rest_detected = false;
        return PYRO_WARNING;
    }

    if (_config.rest_bias_est_enabled)
    {
        _rest_acc_lp.process(acc.data(), _state.rest_acc_lp.data(), dt);
        const vector3_t deviation = acc - _state.rest_acc_lp;
        const float squared_deviation = square(deviation[0]) +
            square(deviation[1]) + square(deviation[2]);
        if (!_state.rest_gyr_ok || squared_deviation >= square(_config.rest_th_acc))
        {
            _state.rest_t = 0.0f;
            _state.rest_detected = false;
        }
        else
        {
            _state.rest_t += dt;
            _state.rest_detected = _state.rest_t >= _config.rest_min_t;
        }
    }

    // 两处旋转都只用到向量结果，直接按四元数旋转，不构造 3x3 矩阵
    const vector_t<3> acc_inertial = rotate(_state.gyr_quat, acc);
    _acc_lp.process(acc_inertial.data(), _state.last_acc_lp.data(), dt);
    vector_t<3> acc_earth = rotate(_state.acc_quat, _state.last_acc_lp);
    const float earth_magnitude = norm(acc_earth);
    if (!std::isfinite(earth_magnitude) ||
        earth_magnitude <= std::numeric_limits<float>::min())
        return PYRO_WARNING;
    acc_earth /= earth_magnitude;

    const float z = std::clamp(acc_earth[2], -1.0f, 1.0f);
    const float qw = sqrtf(0.5f * (z + 1.0f));
    quaternion_t correction{0.0f, 1.0f, 0.0f, 0.0f};
    if (qw > 1.0e-6f)
        correction = {qw, 0.5f * acc_earth[1] / qw,
                       -0.5f * acc_earth[0] / qw, 0.0f};
    _state.acc_quat = normalized(multiply(correction, _state.acc_quat));

    if (_config.motion_bias_est_enabled || _config.rest_bias_est_enabled)
        return _update_bias(acc_earth, dt);
    return PYRO_OK;
}

status_t vqf_t::_prepare_bias(float dt)
{
    if (dt == _bias_dt)
        return PYRO_OK;
    bias_noise_t noise;
    noise.variance = _coeffs.bias_variance_rate * dt;
    if (!std::isfinite(noise.variance) || noise.variance <= 0.0f)
        return PYRO_PARAM_ERROR;
    const float motion_p = _coeffs.bias_motion_p;
    const float rest_p = _coeffs.bias_rest_p;
    noise.motion = motion_p * (motion_p / noise.variance + 1.0f);
    noise.vertical = noise.motion / std::max(_config.bias_vertical_forgetting_factor, 1.0e-10f);
    noise.rest = rest_p * (rest_p / noise.variance + 1.0f);
    if (!std::isfinite(noise.motion) || !std::isfinite(noise.vertical) || !std::isfinite(noise.rest))
        return PYRO_PARAM_ERROR;
    _bias_noise = noise;
    _bias_dt = dt;
    return PYRO_OK;
}

status_t vqf_t::_update_bias(const vector3_t &acc_earth, float dt)
{
    matrix_t<3, 3> H;
    vector_t<2> bias_lp;
    if (_config.motion_bias_est_enabled)
    {
        H = rotation(multiply(_state.acc_quat, _state.gyr_quat));
        const vector3_t rotated_bias = H * _bias_kf.get_state();
        bias_lp = {rotated_bias[0], rotated_bias[1]};
        _motion_rotation_lp.process(H.data(), H.data(), dt);
        _motion_bias_lp.process(bias_lp.data(), bias_lp.data(), dt);
    }
    if (dt == 0.0f)
        return PYRO_OK;

    // 零偏为随机游走，F=I；达到初始不确定度后停止增加对角方差。
    // Random-walk bias, F=I; stop increasing each diagonal at initial variance.
    bias_kf_t::covariance_t Q;
    const auto &P = _bias_kf.get_covariance();
    for (std::size_t i = 0; i < 3; ++i)
        if (P(i, i) < _coeffs.bias_p0)
            Q(i, i) = _bias_noise.variance;
    status_t result = _bias_kf.predict(_bias_kf.get_state(), identity(), Q);
    if (result != PYRO_OK)
        return result;

    vector3_t residual;
    bias_kf_t::measurement_noise_t W;
    if (_state.rest_detected && _config.rest_bias_est_enabled)
    {
        H = identity();
        residual = _state.rest_gyr_lp - _bias_kf.get_state();
        W = identity(_bias_noise.rest);
    }
    else if (_config.motion_bias_est_enabled)
    {
        residual = vector3_t{-acc_earth[1] / dt + bias_lp[0],
                               acc_earth[0] / dt + bias_lp[1], 0.0f} -
                   H * _bias_kf.get_state();
        W(0, 0) = _bias_noise.motion;
        W(1, 1) = _bias_noise.motion;
        W(2, 2) = _bias_noise.vertical;
    }
    else
        return PYRO_OK;

    const float clip = _config.bias_clip * deg_to_rad;
    for (float &value : residual)
        value = std::clamp(value, -clip, clip);

    // 通过 KF 的状态增量策略执行 VQF 零偏限幅，保留核心协方差和诊断。
    // Clip the bias via KF's delta policy, preserving covariance/diagnostics.
    return _bias_kf.correct(residual, H, W, nullptr,
        [this, clip](bias_kf_t::correction_t &correction)
        {
            if (matrix::multiply(matrix::view(correction.K),
                                 matrix::view(correction.y),
                                 matrix::view(correction.dx)) != ARM_MATH_SUCCESS)
                return PYRO_ERROR;
            for (std::size_t i = 0; i < 3; ++i)
                correction.dx[i] =
                    std::clamp(_bias_kf.get_state()[i] + correction.dx[i], -clip, clip) -
                    _bias_kf.get_state()[i];
            correction.use_dx = true;
            return PYRO_OK;
        });
}

vqf_t::state_t vqf_t::get_state() const
{
    state_t result;
    result.quat = multiply(_state.acc_quat, _state.gyr_quat);
    result.bias = _bias_kf.get_state();
    result.bias_covariance = _bias_kf.get_covariance();
    result.rest_detected = _state.rest_detected;
    result.rest_t = _state.rest_t;
    result.rest_gyr_lp = _state.rest_gyr_lp;
    result.rest_acc_lp = _state.rest_acc_lp;
    result.initialized = _bias_kf.is_initialized();
    result.consumed_gyr = _state.consumed_gyr;
    result.consumed_acc = _state.consumed_acc;
    result.bias_diagnostics = _bias_kf.get_diagnostics();
    float bound = 0.0f;
    for (std::size_t row = 0; row < 3; ++row)
    {
        float row_sum = 0.0f;
        for (std::size_t col = 0; col < 3; ++col)
            row_sum += std::fabs(result.bias_covariance(row, col));
        bound = std::max(bound, row_sum);
    }
    result.bias_sigma = sqrtf(std::min(bound, _coeffs.bias_p0));
    return result;
}

} // namespace pyro
