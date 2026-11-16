/**
 * @file    imu_config.h
 * @brief   产品编译期配置：器件采样、融合参数、航向零点门控、发布与任务资源。
 *
 * 归属产品装配层，是"这个产品怎么装、怎么采、怎么发"的唯一真源。
 * 驱动（PYRo/Component/IMU）与算法（PYRo/Algorithm）都不读本文件：
 * 它们的参数由 ins_service 在 init() 时装配进去，换产品只改这里
 * （装配方向在 imu_mount.h）。
 *
 * 参考结构：Wheel_leg 的 `ins_config_t`（产品层把配置交给 INS 组件）。
 * 这里用编译期常量 + 两个构造函数替代运行时结构体传递，避免同一份参数
 * 在多处复制。
 */

#ifndef IMU_MODULE_CONFIG_IMU_CONFIG_H
#define IMU_MODULE_CONFIG_IMU_CONFIG_H

#include <cstdint>

#include "imu_base_drv.h"
#include "pyro_algo_vqf.h"

namespace imu_module
{
namespace config
{

/* ============================ 器件与采样 ============================
 * 两路 ODR 相同（驱动按同一时间戳合并）。1920 Hz 是实测 1 ms 预算下的取值：
 * update() 平均 518 µs、峰值 793 µs（Release），7680 Hz 需先压低单批成本。
 * 样本环形缓冲容量在驱动内（64 帧），与 ODR 的匹配关系见 lsm6dsv16x_drv.h。 */
constexpr uint16_t kAccRangeG    = 4U;     /* ±g */
constexpr uint16_t kAccOdrHz     = 1920U;
constexpr uint16_t kGyroRangeDps = 2000U;  /* ±°/s */
constexpr uint16_t kGyroOdrHz    = 1920U;
constexpr bool     kFifoEnabled  = true;   /* false = 输出寄存器直读回退路径 */

/* 上电标定：默认关闭，零偏交给 VQF 的静止/运动联合估计。
 * 若开启，标定期间器件必须静止，kCalibrateTimes 个样本后结束
 * （1920 Hz 下 500 个约 260 ms）。 */
constexpr bool     kCalibrateOnBoot = false;
constexpr uint16_t kCalibrateTimes  = 500U;

/* ============================ 航向零点 ============================
 * 6 轴融合的航向不可观测，yaw 零点必须由产品定义。三项条件同时成立才建立：
 *   ① 可信重力方向已建立；② 连续静止已确认；③ 零偏标准差已收敛。
 * 另加超时兜底，避免运动开机时永远给不出可信 yaw。
 * 注意量纲：bias_sigma 是 rad/s。 */
constexpr float    kHeadingBiasSigmaRps = 0.005f; /* rad/s，≈0.29 °/s */
constexpr uint32_t kHeadingTimeoutRounds = 5000U; /* 5 s @1 kHz 融合轮次 */

/* ============================ 发布（协议 v1） ============================ */
constexpr uint8_t  kNodeId        = 1U;    /* 0..31，CAN base_id = 0x300 + id*0x10 */
constexpr bool     kPublishCan    = true;
constexpr bool     kPublishUart   = true;
constexpr uint32_t kPublishPeriodMs = 1U;  /* 1 kHz */

/* 台架自测开关：true 时把 FDCAN1 切到 internal loopback（硬件自答 ACK），
 * 发布层会轮询取回自己发的三帧并逐字节比对，用 imu_pub_can_rx / imu_pub_can_rx_bad
 * 观察。正常运行必须为 false（回环模式下帧不出芯片）。 */
constexpr bool     kCanLoopbackSelfTest = false;

/* ============================ 任务与资源 ============================
 * 两个常驻任务都在 FreeRTOS 堆（12 KB）里。层级优先级：采集/融合高于发布，
 * 发布可以晚一拍，采样不能积压。
 * 发布任务只做"取快照 → 编码 → 提交"，调用深度浅，384 词（1.5 KB）留有余量；
 * 采集/融合任务承载驱动等待与 ESKF，保留 1024 词（4 KB）。
 * 两者的实际余量由 imu_service_ins_stack_free / imu_pub_stack_free 观测。 */
constexpr uint16_t kInsTaskStackWords       = 1024U;
constexpr uint16_t kPublisherTaskStackWords = 384U;
constexpr uint32_t kCpuMhz = 170U;         /* DWT 周期 → 微秒换算 */

/* 板级绑定：片选由产品装配层决定（CubeMX 中 PA4 初始为高）。 */
/* 这两个常量放在 ins_service.cpp，因为它们需要 CubeMX 的 GPIO 类型。 */

/* ============================ 参数装配 ============================ */

/** 驱动侧配置：量程、ODR、FIFO、上电标定。
 *  上电零偏沿用驱动 imu_cali_t 的默认值；开启 kCalibrateOnBoot 时会被覆盖。 */
inline imu_cfg_t driver_config()
{
    imu_cfg_t cfg{};
    cfg.calibrate_flag  = kCalibrateOnBoot;
    cfg.calibrate_times = kCalibrateTimes;
    cfg.acc_range       = kAccRangeG;
    cfg.acc_freq        = kAccOdrHz;
    cfg.gyro_range      = kGyroRangeDps;
    cfg.gyro_freq       = kGyroOdrHz;
    cfg.fifo_en         = kFifoEnabled;
    return cfg;
}

/** 融合参数。门限按本板静止实测噪声设定：
 *  驱动只做单位换算与零偏扣除，静止时角速度残余约 1 °/s，故 rest_th_gyr 取 5 °/s；
 *  加速度静止波动约 0.2~0.3 m/s²，rest_th_acc 取 1.0 留余量。
 *  bias_clip 必须 ≥ rest_th_gyr，否则低通值被提前截断、静止永不成立。
 *  acc_update_period = 2 ms：倾斜/静止观测约 500 Hz，陀螺仍按全部样本传播。 */
inline pyro::vqf_t::config_t vqf_config()
{
    pyro::vqf_t::config_t cfg;
    cfg.tau_acc      = 0.4f;   /* s，加速度倾斜校正时间常数（默认 3.0 偏慢） */
    cfg.rest_min_t   = 1.0f;   /* s，静止确认时间 */
    cfg.rest_th_gyr  = 5.0f;   /* °/s */
    cfg.rest_th_acc  = 1.0f;   /* m/s² */
    cfg.bias_clip    = 5.0f;   /* °/s，与 rest_th_gyr 匹配 */
    cfg.motion_bias_est_enabled = true;
    cfg.rest_bias_est_enabled   = true;
    cfg.acc_update_period = 0.002f;
    return cfg;
}

} // namespace config
} // namespace imu_module

#endif /* IMU_MODULE_CONFIG_IMU_CONFIG_H */
