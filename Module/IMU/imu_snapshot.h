/**
 * @file    imu_snapshot.h
 * @brief   统一输出快照：一次融合轮次的完整结果 + 该次结果的解包参数。
 *
 * 对外（Communication 层）只有这一个数据结构。它与参考实现 Wheel_leg 的
 * "13 个语义化 getter" 形成对比：那些 getter 各自读私有缓存，两次调用之间
 * 任务可能已翻拍，跨字段不同拍；协议要求"同组数据"时这就是错数据。
 * 这里一次取全量，跨字段天然同拍。
 */

#ifndef IMU_MODULE_IMU_IMU_SNAPSHOT_H
#define IMU_MODULE_IMU_IMU_SNAPSHOT_H

#include <cstdint>

namespace imu_module
{

/** 反向封包所需的解包参数。
 *  与测量值在【同一次快照】里取出，保证反向换算用的是产生这些物理量的那组
 *  量程/灵敏度/零偏/装配方向——切换参数时不会出现"新参数换算旧样本"。 */
struct imu_publish_params_t
{
    float   acc_mps2_per_lsb;  /**< Sa = acc_range_g * 9.80665 / 32768 */
    float   gyro_rps_per_lsb;  /**< Sg = gyro_dps_per_lsb * pi / 180 */
    float   gyro_bias_dev[3];  /**< 驱动解包时扣除的静态零偏 b，器件系，rad/s */
    uint8_t inv_idx[3];        /**< Mᵀ（安装系 → 器件系）轴索引 */
    bool    inv_neg[3];        /**< Mᵀ 对应轴的符号 */
};

/** 统一输出快照。
 *  四元数、加速度、角速度、零偏均为【安装坐标系】且共用同一航向零点；
 *  欧拉角由归零后的四元数导出（欧拉角不是线性量，必须最后算）。 */
struct imu_snapshot_t
{
    uint32_t sequence;       /**< 融合轮次序号（内部诊断用，不上线） */
    uint32_t timestamp_ms;   /**< 本拍完成时刻，HAL tick */
    float    quat[4];        /**< [w,x,y,z]，安装系 → Z-up，航向已归零 */
    float    euler[3];       /**< roll/pitch/yaw，°（yaw 相对产品零点） */
    float    accel[3];       /**< 安装系，m/s² */
    float    gyro[3];        /**< 安装系，rad/s */
    float    bias[3];        /**< 安装系，rad/s */
    float    rest_t;         /**< 连续静止计时，s */
    bool     rest;           /**< 当前判定为静止 */
    bool     valid;          /**< 可信重力方向已建立；false 时 quat 无意义 */
    bool     heading_zeroed; /**< yaw 零点已建立；false 时 yaw 无产品含义 */
    uint8_t  heading_quality;/**< 0=未建立 1=静止且零偏收敛 2=超时兜底 */
    imu_publish_params_t params;
};

} // namespace imu_module

#endif /* IMU_MODULE_IMU_IMU_SNAPSHOT_H */
