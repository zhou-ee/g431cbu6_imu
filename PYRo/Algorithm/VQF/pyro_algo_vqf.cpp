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
constexpr float gravity = 9.80665f;
constexpr float vibration_tau = 0.05f;
constexpr float min_acc_confidence = 0.2f;
constexpr float max_correction_angle = 0.1f;

float square(float value) { return value * value; }

float norm(const vector_t<3> &value)
{
    return sqrtf(square(value[0]) + square(value[1]) + square(value[2]));
}

vqf_t::quaternion_t multiply(const vqf_t::quaternion_t &a, const vqf_t::quaternion_t &b)
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

vector_t<3> rotate(const vqf_t::quaternion_t &q, const vector_t<3> &v)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float tx = 2.0f * (y * v[2] - z * v[1]);
    const float ty = 2.0f * (z * v[0] - x * v[2]);
    const float tz = 2.0f * (x * v[1] - y * v[0]);
    return {v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx};
}

template <std::size_t N> matrix_t<N, N> identity(float scale = 1.0f)
{
    matrix_t<N, N> result;
    for (std::size_t i = 0; i < N; ++i)
        result(i, i) = scale;
    return result;
}

vqf_t::quaternion_t rotation_step(const vector_t<3> &angle)
{
    const float magnitude = norm(angle);
    if (magnitude < 1.0e-6f)
        return normalized({1.0f, 0.5f * angle[0], 0.5f * angle[1], 0.5f * angle[2]});
    const float scale = sinf(0.5f * magnitude) / magnitude;
    return {cosf(0.5f * magnitude), scale * angle[0], scale * angle[1], scale * angle[2]};
}

// SO(3) 左雅可比，用于旋转积分及全局误差注入后的协方差重置。
// J_l(v)=I+(1-cos|v|)/|v|²[v]x+(|v|-sin|v|)/|v|³[v]x²。
matrix_t<3, 3> left_jacobian(const vector_t<3> &angle)
{
    const float squared = square(angle[0]) + square(angle[1]) + square(angle[2]);
    float first = 0.5f - squared / 24.0f;
    float second = 1.0f / 6.0f - squared / 120.0f;
    if (squared > 1.0e-4f)
    {
        const float magnitude = sqrtf(squared);
        first = (1.0f - cosf(magnitude)) / squared;
        second = (magnitude - sinf(magnitude)) / (squared * magnitude);
    }
    const matrix_t<3, 3> skew{0.0f, -angle[2], angle[1],
                              angle[2], 0.0f, -angle[0],
                              -angle[1], angle[0], 0.0f};
    return identity<3>() + skew * first + (skew * skew) * second;
}

// 将单位方向转向 +Z 的最短旋转向量，取水平两维作为倾斜新息。
vector_t<3> inclination_error(const vector_t<3> &direction)
{
    const float horizontal = sqrtf(square(direction[0]) + square(direction[1]));
    const float angle = atan2f(horizontal, direction[2]);
    if (horizontal < 1.0e-6f)
        return direction[2] < 0.0f ? vector_t<3>{3.14159265358979323846f, 0.0f, 0.0f}
                                   : vector_t<3>{direction[1], -direction[0], 0.0f};
    return {angle * direction[1] / horizontal, -angle * direction[0] / horizontal, 0.0f};
}

template <typename Sample> bool valid_stream(const sample_stream_t<Sample> &stream)
{
    return stream.count == 0 ||
        (stream.sample && stream.count <= stream.capacity && stream.head < stream.capacity);
}
} // namespace

status_t vqf_t::configure(const config_t &config)
{
    const float positive[] = {
        config.tau_acc, config.bias_sigma_init, config.bias_forgetting_time,
        config.bias_clip, config.bias_sigma_rest, config.rest_filter_tau,
        config.rest_th_gyr, config.rest_th_acc, config.attitude_sigma_init,
        config.gyro_noise_density, config.acc_noise, config.acc_norm_tolerance,
        config.acc_vibration_tolerance, config.acc_update_period, config.acc_nis_threshold};
    for (float value : positive)
        if (!matrix::is_finite(value) || value <= 0.0f ||
            !matrix::is_finite(square(value)) || square(value) == 0.0f)
            return PYRO_PARAM_ERROR;
    if (!matrix::is_finite(config.rest_min_t) || config.rest_min_t < 0.0f ||
        !matrix::is_finite(config.acc_recovery_t) || config.acc_recovery_t < 0.0f)
        return PYRO_PARAM_ERROR;
    const float derived[] = {
        square(config.bias_sigma_init * deg_to_rad),
        square(config.attitude_sigma_init * deg_to_rad),
        square(config.gyro_noise_density * deg_to_rad),
        square(config.bias_sigma_rest * deg_to_rad),
        square(0.1f * deg_to_rad) / config.bias_forgetting_time,
        square(config.acc_noise / gravity)};
    for (float value : derived)
        if (!matrix::is_finite(value) || value <= 0.0f)
            return PYRO_PARAM_ERROR;

    const bool first = !_error_kf.is_initialized();
    if (first)
    {
        error_kf_t::covariance_t P;
        for (std::size_t i = 0; i < 3; ++i)
        {
            P(i, i) = derived[1];
            P(i + 3, i + 3) = derived[0];
        }
        error_kf_t::options_t options;
        for (std::size_t i = 0; i < 3; ++i)
        {
            options.min_variance[i] = 1.0e-12f;
            options.min_variance[i + 3] = 1.0e-14f;
        }
        const status_t result = _error_kf.init({}, P, options);
        if (result != PYRO_OK)
            return result;
    }
    const bool rest_changed = config.rest_bias_est_enabled != _config.rest_bias_est_enabled ||
        config.rest_filter_tau != _config.rest_filter_tau || config.rest_min_t != _config.rest_min_t ||
        config.rest_th_gyr != _config.rest_th_gyr || config.rest_th_acc != _config.rest_th_acc ||
        config.bias_clip != _config.bias_clip;
    const bool acc_changed = config.tau_acc != _config.tau_acc ||
        config.acc_noise != _config.acc_noise || config.acc_norm_tolerance != _config.acc_norm_tolerance ||
        config.acc_vibration_tolerance != _config.acc_vibration_tolerance ||
        config.acc_recovery_t != _config.acc_recovery_t ||
        config.acc_nis_threshold != _config.acc_nis_threshold;
    if (rest_changed)
    {
        _rest_gyr_lp.reset();
        _rest_acc_lp.reset();
        _state.rest_gyr_lp = {};
        _state.rest_acc_lp = {};
        _state.rest_t = 0.0f;
        _state.rest_gyr_ok = _state.rest_detected = false;
    }
    if (acc_changed)
    {
        _state.trusted_acc_t = _state.observation_t = 0.0f;
        _acc_lp.reset();
    }
    (void)_acc_lp.configure(config.tau_acc);
    (void)_rest_gyr_lp.configure(config.rest_filter_tau);
    (void)_rest_acc_lp.configure(config.rest_filter_tau);
    _config = config;
    if (first)
        return reset();
    const float clip = _config.bias_clip * deg_to_rad;
    for (float &value : _state.bias)
        value = std::clamp(value, -clip, clip);
    return PYRO_OK;
}

status_t vqf_t::reset(const vector3_t &bias, float sigma)
{
    if (!_error_kf.is_initialized())
        return PYRO_NOT_FOUND;
    if (!matrix::is_finite(bias) || !matrix::is_finite(sigma) || sigma < 0.0f ||
        !matrix::is_finite(square(sigma)))
        return PYRO_PARAM_ERROR;
    const float clip = _config.bias_clip * deg_to_rad;
    for (float value : bias)
        if (std::fabs(value) > clip)
            return PYRO_PARAM_ERROR;
    error_kf_t::covariance_t P;
    for (std::size_t i = 0; i < 3; ++i)
    {
        P(i, i) = square(_config.attitude_sigma_init * deg_to_rad);
        P(i + 3, i + 3) = sigma > 0.0f ? square(sigma) : square(_config.bias_sigma_init * deg_to_rad);
    }
    const status_t result = _error_kf.reset({}, P);
    if (result != PYRO_OK)
        return result;
    _state = {};
    _state.bias = bias;
    _diagnostics = {};
    _acc_lp.reset();
    _rest_gyr_lp.reset();
    _rest_acc_lp.reset();
    err_stage = 0U;
    return PYRO_OK;
}

status_t vqf_t::update(const imu_data_t &data)
{
    err_stage = 0U;
    _state.consumed_gyr = 0;
    _state.consumed_acc = 0;
    if (!_error_kf.is_initialized())
        return PYRO_NOT_FOUND;
    if (!valid_stream(data.gyro) || !valid_stream(data.acc))
    {
        err_stage = 1;
        return PYRO_PARAM_ERROR;
    }
    if (data.gyro.count == 0 && data.acc.count == 0)
        return PYRO_OK;
    if (!std::isfinite(data.timestamp_tick_s) || data.timestamp_tick_s <= 0.0f)
    {
        err_stage = 2;
        return PYRO_PARAM_ERROR;
    }
    if (_state.timestamp_tick_s != 0.0f && _state.timestamp_tick_s != data.timestamp_tick_s)
    {
        err_stage = 3;
        return PYRO_PARAM_ERROR;
    }

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
        {
            err_stage = 4; /* 时间戳倒退 */
            return PYRO_PARAM_ERROR;
        }
        const float dt = clock.initialized ? static_cast<float>(ticks) * data.timestamp_tick_s : 0.0f;
        if (!std::isfinite(dt) || (clock.initialized && ticks > 0 && dt <= 0.0f))
        {
            err_stage = 5;
            return PYRO_PARAM_ERROR;
        }
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
    const vector3_t unbiased = gyr - _state.bias;
    const vector3_t angle = unbiased * dt;
    if (!matrix::is_finite(norm(unbiased)) || !matrix::is_finite(norm(angle)))
        return PYRO_PARAM_ERROR;
    if (dt > 0.0f)
    {
        const quaternion_t next_gyr = normalized(multiply(_state.gyr_quat, rotation_step(angle)));
        // R_true=Exp(δθ_global)R_nom，δθ_dot=-R_nom δb-R_nom n_g。
        // 平均旋转 A=R_start J_l(ω dt)，对恒角速度积分偏置耦合。
        const auto A = rotation(multiply(_state.acc_quat, _state.gyr_quat)) * left_jacobian(angle);
        const auto AAt = A * A.transpose();
        auto F = identity<6>();
        error_kf_t::covariance_t Q;
        const float bias_rate = square(0.1f * deg_to_rad) / _config.bias_forgetting_time;
        const float gyro_variance = square(_config.gyro_noise_density * deg_to_rad) * dt;
        const float bias_variance = bias_rate * dt;
        for (std::size_t row = 0; row < 3; ++row)
        {
            Q(row, row) = gyro_variance;
            Q(row + 3, row + 3) = bias_variance;
            for (std::size_t col = 0; col < 3; ++col)
            {
                F(row, col + 3) = -A(row, col) * dt;
                Q(row, col) += AAt(row, col) * bias_variance * square(dt) / 3.0f;
                Q(row, col + 3) = Q(col + 3, row) = -0.5f * A(row, col) * bias_variance * dt;
            }
        }
        const status_t result = _error_kf.predict({}, F, Q);
        _diagnostics = _error_kf.get_diagnostics();
        if (result != PYRO_OK)
            return result;
        _state.gyr_quat = next_gyr;
    }
    if (_config.rest_bias_est_enabled)
    {
        (void)_rest_gyr_lp.process(gyr.data(), _state.rest_gyr_lp.data(), dt);
        const vector3_t deviation = gyr - _state.rest_gyr_lp;
        const float clip = _config.bias_clip * deg_to_rad;
        _state.rest_gyr_ok = norm(deviation) < _config.rest_th_gyr * deg_to_rad &&
            std::fabs(_state.rest_gyr_lp[0]) <= clip &&
            std::fabs(_state.rest_gyr_lp[1]) <= clip &&
            std::fabs(_state.rest_gyr_lp[2]) <= clip;
        if (!_state.rest_gyr_ok)
        {
            _state.rest_t = 0.0f;
            _state.rest_detected = false;
        }
    }
    return PYRO_OK;
}

void vqf_t::_reject_acc()
{
    _state.acc_rejected = true;
    ++_state.acc_rejections;
    _state.trusted_acc_t = _state.observation_t = 0.0f;
    _state.rest_t = 0.0f;
    _state.rest_detected = false;
    // 恢复时重新积累可信历史，冲击样本的低通尾迹在此清除。
    _acc_lp.reset();
}

status_t vqf_t::_update_acc(const vector3_t &acc, float dt)
{
    const float magnitude = norm(acc);
    _state.acc_confidence = 0.0f;
    if (magnitude <= std::numeric_limits<float>::min())
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    const float norm_error = std::fabs(magnitude - gravity);
    if (norm_error > 3.0f * _config.acc_norm_tolerance)
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    const vector3_t acc_inertial = rotate(_state.gyr_quat, acc);
    if (!_state.reference_initialized)
    {
        _state.acc_reference = acc_inertial;
        _state.reference_initialized = true;
    }
    // 50 ms 能量窗口在近惯性系评估，真实转动导致的重力投影变化已补偿。
    const float gain = dt / (vibration_tau + dt);
    const vector3_t highpass = acc_inertial - _state.acc_reference;
    const float energy = square(highpass[0]) + square(highpass[1]) + square(highpass[2]);
    const float next_variance = _state.vibration_variance + gain * (energy - _state.vibration_variance);
    if (!matrix::is_finite(energy) || !matrix::is_finite(next_variance))
    {
        _state.reference_initialized = false;
        _state.vibration_variance = 0.0f;
        _reject_acc();
        return PYRO_WARNING;
    }
    _state.acc_reference += highpass * gain;
    _state.vibration_variance = next_variance;
    float confidence = 1.0f / (1.0f + square(norm_error / _config.acc_norm_tolerance) +
        _state.vibration_variance / square(_config.acc_vibration_tolerance));
    bool direction_rejected = false;
    if (_state.tilt_initialized)
    {
        const auto raw_direction = rotate(_state.acc_quat, acc_inertial / magnitude);
        const auto raw_error = inclination_error(raw_direction);
        const auto &P = _error_kf.get_covariance();
        const float variance = P(0, 0) + P(1, 1) + 2.0f * square(_config.acc_noise / magnitude);
        const float normalized_error = (square(raw_error[0]) + square(raw_error[1])) / variance;
        confidence /= 1.0f + normalized_error / _config.acc_nis_threshold;
        direction_rejected = normalized_error > _config.acc_nis_threshold;
    }
    _state.acc_confidence = std::clamp(confidence, 0.0f, 1.0f);
    if (!matrix::is_finite(confidence) ||
        confidence < min_acc_confidence || direction_rejected)
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    _state.acc_rejected = false;
    if (_config.rest_bias_est_enabled)
    {
        (void)_rest_acc_lp.process(acc.data(), _state.rest_acc_lp.data(), dt);
        const auto deviation = acc - _state.rest_acc_lp;
        if (_state.rest_gyr_ok && norm(deviation) < _config.rest_th_acc &&
            norm_error < _config.rest_th_acc)
        {
            _state.rest_t += dt;
            _state.rest_detected = _state.rest_t >= _config.rest_min_t;
        }
        else
        {
            _state.rest_t = 0.0f;
            _state.rest_detected = false;
        }
    }
    (void)_acc_lp.process(acc_inertial.data(), _state.last_acc_lp.data(), dt);
    _state.trusted_acc_t += dt;
    _state.observation_t += dt;
    if (_state.trusted_acc_t < _config.acc_recovery_t)
    {
        _state.acc_rejected = true;
        ++_state.acc_rejections;
        return PYRO_WARNING;
    }
    if (!_state.tilt_initialized)
        return _initialize_tilt();
    if (_state.observation_t < _config.acc_update_period)
        return PYRO_OK;
    const float interval = _state.observation_t;
    _state.observation_t = 0.0f;
    const status_t result = _correct_acc(interval);
    if (result == PYRO_WARNING)
        _reject_acc();
    if (result != PYRO_OK)
        return result;
    if (_state.rest_detected && _config.rest_bias_est_enabled)
        return _correct_rest(interval);
    return PYRO_OK;
}

status_t vqf_t::_initialize_tilt()
{
    const float magnitude = norm(_state.last_acc_lp);
    if (!matrix::is_finite(magnitude) || magnitude <= std::numeric_limits<float>::min())
    {
        _reject_acc();
        return PYRO_WARNING;
    }
    const auto direction = rotate(_state.acc_quat, _state.last_acc_lp / magnitude);
    const auto next_acc = normalized(multiply(rotation_step(inclination_error(direction)), _state.acc_quat));
    auto P = _error_kf.get_covariance();
    // 初始重力对齐确定两个倾斜自由度，全局航向保留初始不确定度。
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 6; ++col)
            P(row, col) = P(col, row) = 0.0f;
    P(0, 0) = P(1, 1) = square(_config.acc_noise / magnitude) / square(_state.acc_confidence);
    P(2, 2) = square(_config.attitude_sigma_init * deg_to_rad);
    const status_t result = _error_kf.reset({}, P);
    if (result != PYRO_OK)
        return result;
    _state.acc_quat = next_acc;
    _state.tilt_initialized = true;
    _state.observation_t = 0.0f;
    return PYRO_OK;
}

status_t vqf_t::_correct_acc(float interval)
{
    const float magnitude = norm(_state.last_acc_lp);
    if (!matrix::is_finite(magnitude) || magnitude <= std::numeric_limits<float>::min())
        return PYRO_WARNING;
    const auto direction = rotate(_state.acc_quat, _state.last_acc_lp / magnitude);
    const auto residual = inclination_error(direction);
    error_kf_t::observation_t H;
    H(0, 0) = H(1, 1) = 1.0f;
    error_kf_t::measurement_noise_t R;
    // 低通观测相关：按 tau/实际观测间隔放大 R，限制重复使用相邻滤波样本的信息量。
    // 这是保守的相关时间近似，硬件噪声标定及回放用于进一步调参。
    const float correlation_scale = std::max(1.0f, _config.tau_acc / interval);
    R(0, 0) = R(1, 1) = square(_config.acc_noise / magnitude) * correlation_scale /
        square(_state.acc_confidence);
    const status_t result = _correct(residual, H, R, true);
    if (result == PYRO_OK)
        ++_state.acc_corrections;
    return result;
}

status_t vqf_t::_correct_rest(float interval)
{
    error_kf_t::observation_t H;
    for (std::size_t i = 0; i < 3; ++i)
        H(i, i + 3) = 1.0f;
    const float variance = square(_config.bias_sigma_rest * deg_to_rad) *
        std::max(1.0f, 2.0f * _config.rest_filter_tau / interval);
    return _correct(_state.rest_gyr_lp - _state.bias, H, identity<3>(variance), false);
}

status_t vqf_t::_gate(error_kf_t::correction_t &correction, bool acceleration)
{
    if (acceleration && correction.diagnostics.nis > _config.acc_nis_threshold)
        return PYRO_WARNING;
    if (acceleration)
    {
        // 六轴重力观测修正全局水平倾斜，航向误差维持陀螺传播。
        for (std::size_t col = 0; col < 3; ++col)
            correction.K(2, col) = 0.0f;
        if (!_config.motion_bias_est_enabled || _state.rest_detected)
            for (std::size_t row = 3; row < 6; ++row)
                for (std::size_t col = 0; col < 3; ++col)
                    correction.K(row, col) = 0.0f;
    }
    error_kf_t::state_t delta;
    if (matrix::multiply(matrix::view(correction.K), matrix::view(correction.y),
                         matrix::view(delta)) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    const float angle = norm({delta[0], delta[1], delta[2]});
    if (!matrix::is_finite(delta) || !matrix::is_finite(angle))
        return PYRO_ERROR;
    const float scale = angle > max_correction_angle ? max_correction_angle / angle : 1.0f;
    correction.K *= scale;
    const float clip = _config.bias_clip * deg_to_rad;
    for (std::size_t row = 3; row < 6; ++row)
    {
        const float change = delta[row] * scale;
        const float candidate = _state.bias[row - 3] + change;
        if (change == 0.0f || std::fabs(candidate) <= clip)
            continue;
        const float bounded = std::clamp(candidate, -clip, clip);
        const float row_scale = std::clamp((bounded - _state.bias[row - 3]) / change, 0.0f, 1.0f);
        for (std::size_t col = 0; col < 3; ++col)
            correction.K(row, col) *= row_scale;
    }
    // 核心根据调整后的 K 同时执行状态及 Joseph 协方差更新。
    return PYRO_OK;
}

status_t vqf_t::_correct(const vector3_t &residual, const error_kf_t::observation_t &H,
                         const error_kf_t::measurement_noise_t &R, bool acceleration)
{
    const auto prior = _error_kf.get_covariance();
    error_kf_t::measurement_mask_t mask{true, true, !acceleration};
    const status_t result = _error_kf.correct(residual, H, R, &mask,
        [this, acceleration](error_kf_t::correction_t &correction)
        { return _gate(correction, acceleration); });
    _diagnostics = _error_kf.get_diagnostics();
    if (acceleration)
        _state.acc_nis = _diagnostics.nis;
    if (result != PYRO_OK)
        return result;
    const status_t injected = _inject_error();
    if (injected != PYRO_OK)
    {
        // 注入失败时恢复修正前协方差，名义姿态和零偏仍保持原值。
        (void)_error_kf.reset({}, prior);
        _diagnostics.status = injected;
    }
    return injected;
}

status_t vqf_t::_inject_error()
{
    const auto &delta = _error_kf.get_state();
    const vector3_t angle{delta[0], delta[1], delta[2]};
    const auto next_acc = normalized(multiply(rotation_step(angle), _state.acc_quat));
    const auto next_bias = _state.bias + vector3_t{delta[3], delta[4], delta[5]};
    if (!matrix::is_finite(next_acc) || !matrix::is_finite(next_bias))
        return PYRO_ERROR;
    auto G = identity<6>();
    const auto J = left_jacobian(angle);
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 3; ++col)
            G(row, col) = J(row, col);
    const auto P = G * _error_kf.get_covariance() * G.transpose();
    const status_t result = _error_kf.reset({}, P);
    if (result != PYRO_OK)
        return result;
    _state.acc_quat = next_acc;
    _state.bias = next_bias;
    return PYRO_OK;
}

vqf_t::state_t vqf_t::get_state() const
{
    state_t result;
    result.quat = multiply(_state.acc_quat, _state.gyr_quat);
    result.bias = _state.bias;
    result.error_covariance = _error_kf.get_covariance();
    float bound = 0.0f;
    for (std::size_t row = 0; row < 3; ++row)
    {
        float sum = 0.0f;
        for (std::size_t col = 0; col < 3; ++col)
        {
            result.bias_covariance(row, col) = result.error_covariance(row + 3, col + 3);
            sum += std::fabs(result.bias_covariance(row, col));
        }
        bound = std::max(bound, sum);
    }
    result.bias_sigma = sqrtf(bound);
    result.rest_detected = _state.rest_detected;
    result.rest_t = _state.rest_t;
    result.rest_gyr_lp = _state.rest_gyr_lp;
    result.rest_acc_lp = _state.rest_acc_lp;
    result.initialized = _state.tilt_initialized;
    result.consumed_gyr = _state.consumed_gyr;
    result.consumed_acc = _state.consumed_acc;
    result.acc_confidence = _state.acc_confidence;
    result.acc_nis = _state.acc_nis;
    result.acc_rejected = _state.acc_rejected;
    result.acc_corrections = _state.acc_corrections;
    result.acc_rejections = _state.acc_rejections;
    result.bias_diagnostics = _diagnostics;
    return result;
}

} // namespace pyro
