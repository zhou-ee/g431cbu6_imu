#include "imu_base_drv.h"
#include <cmath>

imu_base_drv::~imu_base_drv() = default;

/**
 * @brief 陀螺零偏 + 重力模长标定
 * @note  依赖派生类 read() 的语义："取回一批样本，陀螺写入 gyro_sample、
 *        加速度写入 acc 流，两者未被消费的数量体现在 acc.count / gyro.count"。
 *        本函数不关心 read() 的返回值，只按两路计数判断本批有没有新数据。
 *        标定期间器件需保持静止。两路独立计数，各自攒够 target 个样本为止
 *        （陀螺/加速度同速率时同步完成；7680 Hz 下 500 个约 65 ms）。
 *        本函数会**消费**样本（读后调 assume_*），避免下一轮重复累加同一批。
 *        单位：Gyro 为 rad/s；Accel 为 m/s²，故静止时 g_norm ≈ 9.80665。
 */
bool imu_base_drv::calibrate() {
    const uint16_t target = (_data.cfg.calibrate_times > 0U) ? _data.cfg.calibrate_times : 2000U;
    const imu_cali_t previous_cali = _data.cali;
    const imu_direct_t previous_direct = _data.cfg.direct;

    /* 偏置存储在传感器原坐标系。标定读取未扣偏置、未换轴的新样本。 */
    _data.cali.gx_offset = 0.0f;
    _data.cali.gy_offset = 0.0f;
    _data.cali.gz_offset = 0.0f;
    _data.cfg.direct = imu_direct_t::NONE;
    assume_gyro_point(_data.gyro.count);
    assume_acc_point(_data.acc.count);

    double   sum_gyro[3] = {0.0, 0.0, 0.0};
    double   sum_norm    = 0.0;
    uint32_t got_gyro    = 0U;
    uint32_t got_acc     = 0U;
    uint32_t guard       = 0U;

    while (got_gyro < static_cast<uint32_t>(target) || got_acc < static_cast<uint32_t>(target)) {
        const uint32_t previous_gyro_count = got_gyro;
        const uint32_t previous_acc_count = got_acc;
        (void)read(); /* 标定不关心返回值，只看两路 count */

        const uint8_t ng = _data.gyro.count;
        const uint8_t na = _data.acc.count;
        for (uint8_t i = 0U; i < ng && got_gyro < static_cast<uint32_t>(target); ++i) {
            const gyro_sample_t &s = _data.gyro.at(i); /* 逻辑下标，内部处理回绕 */
            sum_gyro[0] += static_cast<double>(s.Gyro[0]);
            sum_gyro[1] += static_cast<double>(s.Gyro[1]);
            sum_gyro[2] += static_cast<double>(s.Gyro[2]);
            ++got_gyro;
        }
        for (uint8_t i = 0U; i < na && got_acc < static_cast<uint32_t>(target); ++i) {
            const acc_sample_t &s = _data.acc.at(i);
            const double ax = s.Accel[0];
            const double ay = s.Accel[1];
            const double az = s.Accel[2];
            sum_norm += std::sqrt(ax * ax + ay * ay + az * az);
            ++got_acc;
        }

        /* 消费掉本批，下一轮才不会被重复累加 */
        assume_gyro_point(ng);
        assume_acc_point(na);
        if (got_gyro != previous_gyro_count || got_acc != previous_acc_count) {
            guard = 0U;
        } else if (++guard > 5000000U) {
            /* 以实际标定进度检测停滞，覆盖仅一路有数据的情况。 */
            _data.cali = previous_cali;
            _data.cfg.direct = previous_direct;
            return false;
        }
    }

    const double inv_g = 1.0 / static_cast<double>(got_gyro);
    const double inv_a = 1.0 / static_cast<double>(got_acc);
    _data.cali.gx_offset = static_cast<float>(sum_gyro[0] * inv_g);
    _data.cali.gy_offset = static_cast<float>(sum_gyro[1] * inv_g);
    _data.cali.gz_offset = static_cast<float>(sum_gyro[2] * inv_g);
    _data.cali.g_norm    = static_cast<float>(sum_norm * inv_a);
    _data.cfg.direct = previous_direct;

    return true;
}
