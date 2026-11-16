#include "pyro_algo_vqf.h"
#include "imu_base_drv.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

using pyro::vqf_t;
using Vec = std::array<double, 3>;
using Quat = std::array<double, 4>;
constexpr double pi = 3.14159265358979323846;
constexpr double gravity = 9.80665;
constexpr double radians = pi / 180.0;
static bool fail_product = false;
static bool fail_injection = false;

extern "C" arm_status __real_arm_mat_mult_f32(const arm_matrix_instance_f32 *,
    const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
extern "C" arm_status __wrap_arm_mat_mult_f32(const arm_matrix_instance_f32 *a,
    const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *out)
{
    /* 濞夈劌鍙嗛惄顔界垼閿涙俺顕ゅ顔芥暈閸忋儳娈戦崸妤勭箥缁?G璺疨璺疓宀偓 闁插瞼顑囨稉鈧▎?3鑴? 娑旀ɑ纭堕敍鍧?= J_l閿涘苯鐨憴鎺戝娑撳绻庨崡鏇氱秴闂冪绱氶妴?
     * 閺€鍦暏閸ф绮ㄩ弸鍕倵閿?G璺疨" 鏉╂瑤閲?6鑴? 娑旀ɑ纭跺韫瑝鐎涙ê婀敍灞炬櫊閸掋倕鍩嗗蹇庣矤 6鑴? 閺€閫涜礋 3鑴?閵?
     * 閸旂娀鈧喎瀹?only 鐠侯垰绶炴稉濠佺瑝娴兼俺袝閸?_update_gyr閿涘苯娲滃銈堢箹闁插瞼娈?3鑴? 娑旀ɑ纭堕崣顏呮降閼?_inject_error閵?*/
    bool injection = fail_injection && a->numRows == 3 && a->numCols == 3 && b->numCols == 3 &&
        a->pData[0] > 0.999f && a->pData[4] > 0.999f && a->pData[8] > 0.999f;
    if (fail_product || injection)
    {
        fail_product = false;
        fail_injection = false;
        return ARM_MATH_ARGUMENT_ERROR;
    }
    return __real_arm_mat_mult_f32(a, b, out);
}

void require(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAILED %s\n", message);
        std::exit(1);
    }
}

Quat product(const Quat &a, const Quat &b)
{
    return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}

Quat conjugate(const Quat &q) { return {q[0], -q[1], -q[2], -q[3]}; }
Quat axis_angle(const Vec &axis, double angle)
{
    const double s = std::sin(angle / 2);
    return {std::cos(angle / 2), s*axis[0], s*axis[1], s*axis[2]};
}
Vec rotate(const Quat &q, const Vec &v)
{
    const auto result = product(product(q, {0, v[0], v[1], v[2]}), conjugate(q));
    return {result[1], result[2], result[3]};
}
Vec angular_rate(const Quat &before, const Quat &after, double dt)
{
    auto step = product(conjugate(before), after);
    if (step[0] < 0)
        for (auto &value : step) value = -value;
    const double s = std::sqrt(step[1]*step[1]+step[2]*step[2]+step[3]*step[3]);
    const double scale = s < 1e-12 ? 2.0 / dt : 2.0 * std::atan2(s, step[0]) / (s * dt);
    return {step[1]*scale, step[2]*scale, step[3]*scale};
}
Quat output_quat(const vqf_t &filter)
{
    const auto q = filter.get_state().quat;
    return {q[0], q[1], q[2], q[3]};
}
double tilt_error(const vqf_t &filter, const Quat &truth = {1, 0, 0, 0})
{
    const auto predicted = rotate(conjugate(output_quat(filter)), {0, 0, 1});
    const auto actual = rotate(conjugate(truth), {0, 0, 1});
    const Vec cross{predicted[1]*actual[2]-predicted[2]*actual[1],
                    predicted[2]*actual[0]-predicted[0]*actual[2],
                    predicted[0]*actual[1]-predicted[1]*actual[0]};
    const double sine = std::sqrt(cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2]);
    const double cosine = predicted[0]*actual[0]+predicted[1]*actual[1]+predicted[2]*actual[2];
    return std::atan2(sine, cosine) / radians;
}

pyro::status_t feed(vqf_t &filter, uint32_t timestamp, const Vec &gyro, const Vec &acc)
{
    gyro_sample_t g{timestamp, {float(gyro[0]), float(gyro[1]), float(gyro[2])}};
    acc_sample_t a{timestamp, {float(acc[0]), float(acc[1]), float(acc[2])}};
    imu_data_t data;
    data.timestamp_tick_s = 1.0e-6f;
    /* SPSC閿涙瓭apacity 韫囧懘銆忔稉?2 閻ㄥ嫬绠撻敍娑樺礋濡茶姤妞?tail 閸撳秶些閸楀疇銆冪粈?1 娑擃亜绶熷☉鍫ｅ瀭閺嶉攱婀伴妴?*/
    data.gyro = {&g, 2U, 0U, 1U};
    data.acc  = {&a, 2U, 0U, 1U};
    const auto result = filter.update(data);
    require(result == pyro::PYRO_OK || result == pyro::PYRO_WARNING, "valid sample update");
    const auto state = filter.get_state();
    require(state.consumed_gyr == 1 && state.consumed_acc == 1, "consumed both streams");
    double squared_norm = 0;
    for (float value : state.quat) squared_norm += double(value)*value;
    require(std::abs(squared_norm - 1) < 3e-6, "unit quaternion");
    return result;
}

vqf_t::config_t config()
{
    vqf_t::config_t result;
    result.tau_acc = 0.4f;
    result.bias_clip = 5.0f;
    result.motion_bias_est_enabled = false;
    result.rest_bias_est_enabled = true;
    return result;
}

void covariance_check(const vqf_t &filter)
{
#ifndef VQF_TEST_BASELINE
    const auto P = filter.get_state().error_covariance;
    for (std::size_t i = 0; i < 6; ++i)
    {
        require(std::isfinite(P(i, i)) && P(i, i) >= 0, "finite positive covariance diagonal");
        for (std::size_t j = 0; j < 6; ++j)
            require(std::abs(P(i, j)-P(j, i)) < 1e-6, "symmetric covariance");
    }
    pyro::matrix_t<6, 6> lower;
    arm_matrix_instance_f32 source, output;
    arm_mat_init_f32(&source, 6, 6, const_cast<float *>(P.data()));
    arm_mat_init_f32(&output, 6, 6, lower.data());
    require(arm_mat_cholesky_f32(&source, &output) == ARM_MATH_SUCCESS, "positive definite joint covariance");
#else
    (void)filter;
#endif
}

void disturbance_replay()
{
    for (int scenario = 0; scenario < 3; ++scenario)
    {
        vqf_t filter;
        require(filter.configure(config()) == pyro::PYRO_OK, "configure disturbance replay");
        double peak = 0;
        double recovery = 0;
        for (int i = 0; i <= 6500; ++i)
        {
            const double t = i * 0.001;
            Vec acc{0, 0, gravity};
            if (t >= 2 && t < 4)
            {
                if (scenario == 0) acc[0] = 2;
                if (scenario == 1)
                {
                    acc[0] = 4 * std::sin(2*pi*80*t);
                    acc[1] = 3 * std::cos(2*pi*65*t);
                    acc[2] += 2 * std::sin(2*pi*90*t);
                }
                if (scenario == 2 && i % 150 < 20) acc = {12, -8, 18};
            }
            feed(filter, uint32_t(i*1000), {}, acc);
            if (t >= 2 && t < 4) peak = std::max(peak, tilt_error(filter));
            if (t >= 4.5) recovery = std::max(recovery, tilt_error(filter));
            if (i % 500 == 0) covariance_check(filter);
        }
        std::printf("replay %d peak_tilt_deg=%.6f recovery_tilt_deg=%.6f\n", scenario, peak, recovery);
#ifndef VQF_TEST_BASELINE
        require(peak < 0.5 && recovery < 0.2, "disturbance rejection and recovery");
        require(filter.get_state().acc_rejections > 0, "disturbance diagnostics");
        require(filter.get_state().acc_corrections > 0, "observation recovery");
#endif
    }
}

#ifndef VQF_TEST_BASELINE
void initial_alignment_and_motion()
{
    for (const Quat initial : {Quat{1, 0, 0, 0}, axis_angle({1, 0, 0}, pi),
                                axis_angle({0, 1, 0}, pi/2)})
    {
        vqf_t filter;
        require(filter.configure(config()) == pyro::PYRO_OK, "configure alignment");
        require(!filter.get_state().initialized, "wait for gravity initialization");
        const auto acc = rotate(conjugate(initial), {0, 0, gravity});
        for (int i = 0; i <= 1500; ++i) feed(filter, uint32_t(i*1000), {}, acc);
        require(filter.get_state().initialized && tilt_error(filter, initial) < 0.02,
                "arbitrary initial pose including inverted");
        covariance_check(filter);
    }
    vqf_t filter;
    require(filter.configure(config()) == pyro::PYRO_OK, "configure rotating IMU");
    Quat before{1, 0, 0, 0};
    double peak = 0;
    for (int i = 0; i <= 10000; ++i)
    {
        const double t = std::max(0.0, i*0.001 - 1);
        const Quat truth = product(product(axis_angle({0, 0, 1}, 1.5*t),
            axis_angle({0, 1, 0}, 0.6*std::sin(2*pi*0.7*t))),
            axis_angle({1, 0, 0}, 0.8*std::sin(2*pi*1.1*t)));
        const auto omega = angular_rate(before, truth, 0.001);
        feed(filter, uint32_t(i*1000), omega, rotate(conjugate(truth), {0, 0, gravity}));
        if (i > 1000) peak = std::max(peak, tilt_error(filter, truth));
        before = truth;
    }
    std::printf("true_rotation peak_tilt_deg=%.6f\n", peak);
    require(peak < 0.05 && filter.get_state().acc_confidence > 0.9f, "preserve true rotation");
    covariance_check(filter);
}

void rest_and_motion_bias()
{
    const Vec bias{0.6*radians, -0.4*radians, 0.3*radians};
    vqf_t filter;
    require(filter.configure(config()) == pyro::PYRO_OK, "configure rest bias");
    for (int i = 0; i <= 16000; ++i)
        feed(filter, uint32_t(i*1000), bias, {0, 0, gravity});
    auto state = filter.get_state();
    require(state.rest_detected, "rest detection");
    for (int axis = 0; axis < 3; ++axis)
        require(std::abs(state.bias[axis]-bias[axis]) / radians < 0.04, "three-axis rest bias convergence");
    require(tilt_error(filter) < 0.2, "rest tilt convergence");
    covariance_check(filter);
    std::printf("rest_bias_deg_s %.6f %.6f %.6f\n", state.bias[0]/radians, state.bias[1]/radians, state.bias[2]/radians);

    auto cfg = config();
    cfg.motion_bias_est_enabled = true;
    cfg.rest_bias_est_enabled = false;
    require(filter.configure(cfg) == pyro::PYRO_OK && filter.reset() == pyro::PYRO_OK, "configure motion bias");
    Quat before{1, 0, 0, 0};
    for (int i = 0; i <= 50000; ++i)
    {
        const double t = std::max(0.0, i*0.001-1);
        const Quat truth = product(axis_angle({0, 1, 0}, 0.7*std::sin(0.7*t)),
                                   axis_angle({1, 0, 0}, 0.8*std::sin(1.3*t)));
        auto omega = angular_rate(before, truth, 0.001);
        for (int axis = 0; axis < 3; ++axis) omega[axis] += bias[axis];
        feed(filter, uint32_t(i*1000), omega, rotate(conjugate(truth), {0, 0, gravity}));
        before = truth;
    }
    state = filter.get_state();
    std::printf("motion_bias_deg_s %.6f %.6f %.6f tilt_deg=%.6f\n", state.bias[0]/radians,
                state.bias[1]/radians, state.bias[2]/radians, tilt_error(filter, before));
    for (int axis = 0; axis < 3; ++axis)
        require(std::abs(state.bias[axis]-bias[axis]) / radians < 0.15, "excited motion bias convergence");
    require(tilt_error(filter, before) < 0.5, "motion bias tilt");
    covariance_check(filter);
}

void noisy_dynamic_replay()
{
    vqf_t filter;
    auto cfg = config();
    cfg.motion_bias_est_enabled = true;
    cfg.rest_th_acc = 1.0f;
    require(filter.configure(cfg) == pyro::PYRO_OK, "configure noisy dynamics");
    std::mt19937 random(42);
    std::normal_distribution<double> accelerometer_noise(0, 0.2);
    std::normal_distribution<double> gyroscope_noise(0, 0.08*radians);
    const Vec bias{0.3*radians, -0.2*radians, 0.15*radians};
    Quat before{1, 0, 0, 0};
    double peak = 0;
    double squared_error = 0;
    int count = 0;
    for (int i = 0; i <= 30000; ++i)
    {
        const double time = i*0.001;
        const double moving = std::max(0.0, time-5.0);
        const Quat truth = product(product(axis_angle({0, 0, 1}, 0.4*moving),
            axis_angle({0, 1, 0}, 0.4*std::sin(0.8*moving))),
            axis_angle({1, 0, 0}, 0.6*std::sin(1.2*moving)));
        auto omega = angular_rate(before, truth, 0.001);
        Vec world_acc{0, 0, gravity};
        if (time >= 9 && time < 12) world_acc[0] += 2.0;
        if (time >= 17 && time < 20)
        {
            world_acc[0] += 4.0*std::sin(2*pi*80*time);
            world_acc[2] += 2.0*std::sin(2*pi*65*time);
        }
        auto acc = rotate(conjugate(truth), world_acc);
        for (int axis = 0; axis < 3; ++axis)
        {
            acc[axis] += accelerometer_noise(random);
            omega[axis] += bias[axis] + gyroscope_noise(random);
        }
        feed(filter, uint32_t(i*1000), omega, acc);
        if (time >= 5)
        {
            const double error = tilt_error(filter, truth);
            peak = std::max(peak, error);
            squared_error += error*error;
            ++count;
        }
        if (i%1000 == 0) covariance_check(filter);
        before = truth;
    }
    std::printf("noisy_dynamics peak_tilt_deg=%.6f rms_tilt_deg=%.6f\n", peak, std::sqrt(squared_error/count));
    require(peak < 1.5 && std::sqrt(squared_error/count) < 0.5, "noisy dynamic accuracy");
    require(filter.get_state().acc_corrections > 100 && filter.get_state().acc_rejections > 100,
            "noisy replay accepts observations and rejects disturbances");
}

void timestamps_and_batching()
{
    vqf_t single, batch;
    require(single.configure(config()) == pyro::PYRO_OK && batch.configure(config()) == pyro::PYRO_OK,
            "configure batching");
    /* SPSC 鐟曚焦鐪?capacity 娑?2 閻ㄥ嫬绠撻敍鍫熷负閻椒鍞弴鍨絿濡槄绱氶妴鍌滄暏 16 濡插鈧弓ead=12 閸掑爼鈧姴娲栫紒鏇窗
     * 闁槒绶?0..9 鐎电懓绨查悧鈺冩倞 12,13,14,15,0,1,2,3,4,5閵?*/
    std::array<gyro_sample_t, 16> gs{};
    std::array<acc_sample_t, 16> as{};
    uint32_t timestamp = UINT32_MAX - 200000;
    for (int base = 0; base < 4000; base += 10)
    {
        for (int i = 0; i < 10; ++i)
        {
            const int slot = (12+i)%16;
            timestamp += uint32_t(900+(base+i)%3*100);
            gs[slot] = {timestamp, {0, 0, 1.0f}};
            as[slot] = {timestamp, {0, 0, float(gravity)}};
            feed(single, timestamp, {0, 0, 1}, {0, 0, gravity});
        }
        imu_data_t data;
        data.timestamp_tick_s = 1e-6f;
        data.gyro = {gs.data(), 16U, 12U, 22U};  /* available = (22-12)&15 = 10 */
        data.acc  = {as.data(), 16U, 12U, 22U};
        const auto status = batch.update(data);
        require(status == pyro::PYRO_OK || status == pyro::PYRO_WARNING, "batch status");
        require(batch.get_state().consumed_acc == 10 && batch.get_state().consumed_gyr == 10,
                "consume wrapped batch");
    }
    for (int i = 0; i < 4; ++i)
        require(single.get_state().quat[i] == batch.get_state().quat[i], "single/batch equivalence");
    covariance_check(batch);

    vqf_t asynchronous;
    require(asynchronous.configure(config()) == pyro::PYRO_OK, "configure asynchronous streams");
    for (int i = 0; i < 10000; ++i)
    {
        gyro_sample_t gyro{uint32_t(i*500), {0, 0, 0.5f}};
        acc_sample_t acc{uint32_t(i*500), {0, 0, float(gravity)}};
        imu_data_t data;
        data.timestamp_tick_s = 1e-6f;
        data.gyro = {&gyro, 2U, 0U, 1U};
        if (i%4 == 0) data.acc = {&acc, 2U, 0U, 1U};
        const auto status = asynchronous.update(data);
        require(status == pyro::PYRO_OK || status == pyro::PYRO_WARNING, "unequal-rate update");
    }
    require(tilt_error(asynchronous) < 0.02 && asynchronous.get_state().initialized, "unequal-rate pose");
    covariance_check(asynchronous);
}

void sensor_tick_rates()
{
    for (const uint32_t step_ticks : {24U, 6U})
    {
        vqf_t filter;
        require(filter.configure(config()) == pyro::PYRO_OK, "configure hardware clock replay");
        const float tick_s = 1.0f / (46080.0f * 1.013f);
        const double dt = double(step_ticks) * tick_s;
        Quat before{1, 0, 0, 0};
        const int count = int(3.0/dt);
        for (int i = 0; i <= count; ++i)
        {
            const double t = i*dt;
            const Quat truth = product(axis_angle({0, 0, 1}, 2*t),
                                       axis_angle({1, 0, 0}, 0.5*std::sin(2*pi*t)));
            const auto omega = angular_rate(before, truth, dt);
            const auto acceleration = rotate(conjugate(truth), {0, 0, gravity});
            gyro_sample_t gyro{uint32_t(i)*step_ticks,
                              {float(omega[0]), float(omega[1]), float(omega[2])}};
            acc_sample_t acc{uint32_t(i)*step_ticks,
                            {float(acceleration[0]), float(acceleration[1]), float(acceleration[2])}};
            imu_data_t data;
            data.timestamp_tick_s = tick_s;
            data.gyro = {&gyro, 2U, 0U, 1U};
            data.acc = {&acc, 2U, 0U, 1U};
            const auto status = filter.update(data);
            require(status == pyro::PYRO_OK || status == pyro::PYRO_WARNING, "hardware clock sample");
            before = truth;
        }
        require(tilt_error(filter, before) < 0.05, "1920/7680 Hz propagation accuracy");
        const double available_time = count*dt - config().acc_recovery_t;
        const double correction_rate = filter.get_state().acc_corrections / available_time;
        std::printf("sensor_ticks=%u correction_rate_hz=%.3f\n", unsigned(step_ticks), correction_rate);
        require(correction_rate > 450 && correction_rate < 510, "approximately 500 Hz acceleration corrections");
        covariance_check(filter);
    }
}

void invalid_inputs_and_reconfiguration()
{
    vqf_t filter;
    imu_data_t data;
    require(filter.update(data) == pyro::PYRO_NOT_FOUND, "unconfigured update");
    auto cfg = config();
    require(filter.configure(cfg) == pyro::PYRO_OK, "configure invalid input tests");
    for (int i = 0; i <= 2000; ++i) feed(filter, uint32_t(i*1000), {}, {0, 0, gravity});
    const auto before = filter.get_state();
    const std::vector<float vqf_t::config_t::*> positive{
        &vqf_t::config_t::tau_acc, &vqf_t::config_t::bias_sigma_init,
        &vqf_t::config_t::bias_forgetting_time, &vqf_t::config_t::bias_clip,
        &vqf_t::config_t::bias_sigma_rest, &vqf_t::config_t::rest_filter_tau,
        &vqf_t::config_t::rest_th_gyr, &vqf_t::config_t::rest_th_acc,
        &vqf_t::config_t::attitude_sigma_init, &vqf_t::config_t::gyro_noise_density,
        &vqf_t::config_t::acc_noise, &vqf_t::config_t::acc_norm_tolerance,
        &vqf_t::config_t::acc_vibration_tolerance, &vqf_t::config_t::acc_update_period,
        &vqf_t::config_t::acc_nis_threshold};
    for (const auto member : positive)
        for (const float invalid : {0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                                   std::numeric_limits<float>::quiet_NaN()})
        {
            auto bad = cfg;
            bad.*member = invalid;
            require(filter.configure(bad) == pyro::PYRO_PARAM_ERROR, "invalid configuration rejected");
            require(filter.get_state().quat == before.quat, "invalid config preserves pose");
        }
    gyro_sample_t gyro{2000000, {0, 0, 0}};
    acc_sample_t acc{2000000, {0, 0, float(gravity)}};
    data.timestamp_tick_s = 1e-6f;
    data.gyro = {&gyro, 2U, 0U, 1U};
    data.acc = {&acc, 2U, 0U, 1U};
    require(filter.update(data) == pyro::PYRO_WARNING && filter.get_state().quat == before.quat,
            "duplicate timestamps skip integration");
    gyro.timestamp--;
    require(filter.update(data) == pyro::PYRO_PARAM_ERROR && filter.get_state().consumed_gyr == 0,
            "backward timestamp remains unconsumed");
    gyro.timestamp = 2001000;
    gyro.Gyro[0] = std::numeric_limits<float>::quiet_NaN();
    require(filter.update(data) == pyro::PYRO_PARAM_ERROR && filter.get_state().consumed_gyr == 0,
            "invalid sample remains unconsumed");
    data.timestamp_tick_s = 2e-6f;
    require(filter.update(data) == pyro::PYRO_PARAM_ERROR, "clock unit changes require reset");
    cfg.tau_acc = 0.8f;
    require(filter.configure(cfg) == pyro::PYRO_OK && filter.get_state().quat == before.quat,
            "live reconfiguration preserves pose");
    require(filter.reset({0.01f, 0, 0}) == pyro::PYRO_OK && !filter.get_state().initialized,
            "reset clears gravity readiness and time base");
    feed(filter, 0, {}, {0, 0, 0});
    for (int i = 1; i <= 1000; ++i) feed(filter, uint32_t(i*1000), {}, {0, 0, gravity});
    require(filter.get_state().initialized, "recover from zero acceleration");
    feed(filter, 1001000, {}, {8e18, 0, 0});
    for (int i = 1002; i <= 2200; ++i) feed(filter, uint32_t(i*1000), {}, {0, 0, gravity});
    require(!filter.get_state().acc_rejected, "recover from extreme finite outlier");
    covariance_check(filter);
}

void numerical_failure_recovery()
{
    vqf_t filter;
    auto cfg = config();
    cfg.rest_bias_est_enabled = false;
    require(filter.configure(cfg) == pyro::PYRO_OK, "configure numerical failure test");
    for (int i = 0; i <= 1500; ++i) feed(filter, uint32_t(i*1000), {}, {0, 0, gravity});
    auto before = filter.get_state();
    gyro_sample_t gyro{1501000, {0.1f, 0, 0}};
    imu_data_t data;
    data.timestamp_tick_s = 1e-6f;
    data.gyro = {&gyro, 2U, 0U, 1U};
    fail_product = true;
    require(filter.update(data) != pyro::PYRO_OK && filter.get_state().consumed_gyr == 0,
            "failed gyro prediction leaves sample for retry");
    require(filter.get_state().quat == before.quat, "failed prediction preserves nominal pose");
    require(filter.update(data) == pyro::PYRO_OK && filter.get_state().consumed_gyr == 1,
            "retry gyro prediction");
    before = filter.get_state();
    acc_sample_t acc{1520000, {0.1f, 0, float(gravity)}};
    data.gyro.tail = 0U;   /* 闄€铻烘祦娓呯┖锛歵ail == head 鍗?0 涓緟娑堣垂鏍锋湰 */
    data.gyro.head = 0U;
    data.acc = {&acc, 2U, 0U, 1U};
    fail_injection = true;
    require(filter.update(data) != pyro::PYRO_OK && !fail_injection, "injection failure reached");
    require(filter.get_state().consumed_acc == 1 && filter.get_state().quat == before.quat,
            "failed injection consumes advanced LP sample and preserves pose");
    require(filter.get_state().error_covariance == before.error_covariance, "failed injection restores covariance");
    feed(filter, 1521000, {}, {0, 0, gravity});
    covariance_check(filter);
}
#endif

int main()
{
    disturbance_replay();
#ifndef VQF_TEST_BASELINE
    initial_alignment_and_motion();
    rest_and_motion_bias();
    noisy_dynamic_replay();
    timestamps_and_batching();
    sensor_tick_rates();
    invalid_inputs_and_reconfiguration();
    numerical_failure_recovery();
    std::printf("PASS VQF/ESKF regressions sizeof(vqf_t)=%zu bytes\n", sizeof(vqf_t));
#endif
    return 0;
}
