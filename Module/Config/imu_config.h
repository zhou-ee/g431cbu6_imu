/**
 * @file    imu_config.h
 * @brief   产品编译期配置：器件采样、融合参数、航向零点门控、发布与任务资源。
 *
 * 归属产品装配层，是"这个产品怎么装、怎么采、怎么发"的唯一真源。
 * 驱动（PYRo/Component/IMU）与算法（PYRo/Algorithm）都不读本文件：
 * 它们的参数由 capture/fusion 在 init() 时装配进去，换产品只改这里
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
 * update() 平均 553 µs、峰值 835 µs（Release，1920 Hz）。
 *
 * 2026-11-17 做过 7680 Hz 的 A/B 实测（诊断后已回退），结论记录在此避免重复踩：
 *   · 双任务拆分生效——采集仍稳在 1000.5 Hz、cap_us max 358 µs，不受融合影响；
 *   · 但 update() 涨到 avg 6528 / max 8333 µs，约为 1920 Hz 的 12 倍
 *     （非线性：陀螺逐样本传播，样本数 ×4 而单拍耗时 ×12）；
 *   · SPSC 缓冲随即被打爆：err_data 以约 1000 次/秒增长，融合实际频率掉到 ~120 Hz。
 * 因此 7680 Hz 在压低单批成本之前不可用。
 * 另：该实验也否证了"算力不足来自运动零偏校准"的猜测——当时
 * motion_bias_est_enabled 已关闭，耗时仍为 8.3 ms。 */
constexpr uint16_t kAccRangeG    = 4U;     /* ±g */
constexpr uint16_t kAccOdrHz     = 1920U;
constexpr uint16_t kGyroRangeDps = 2000U;  /* ±°/s */
constexpr uint16_t kGyroOdrHz    = 1920U;
constexpr bool     kFifoEnabled  = true;   /* false = 输出寄存器直读回退路径 */

/* 单次回读的最大 FIFO 字数。超过则整批丢弃并排空器件 FIFO（见驱动注释）。
 * 1920 Hz 下 1 ms ≈ 4 字（gyro + acc + timestamp），16 字 ≈ 8 倍余量：
 * 既能容忍偶发调度抖动，又能在真正积压时及时止损，避免陈旧样本一次性
 * 涌入压垮软件环形缓冲与算法。设为 0 表示不限制。 */
constexpr uint16_t kFifoMaxWordsPerRead = 16U;

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
 * 三个常驻任务都在 FreeRTOS 堆（12 KB）里：
 *   ins_cap  —— 只做 read()（FIFO → 环形缓冲），1 ms 硬周期，最高优先级
 *   ins_algo —— 只做 update()（融合 + 快照发布），尽力而为，低于采集
 *   imu_pub  —— 只做"取快照 → 编码 → 提交"，与 ins_algo 同级
 *
 * 为什么采集与算法拆开（2026-11-16 版，在 SPSC 环形缓冲落地之后）：
 *   ① 采集周期不再被融合耗时打穿：update() 峰值再高也只降低姿态更新率；
 *   ② 这是提高 ODR 的前提——7680 Hz 下每拍约 7.7 样本/路，单任务必然超预算；
 *   ③ 拆分的安全前提是 sample_stream_t 已改为 SPSC（索引所有权分离 +
 *      publish 屏障），因此【全程不需要任何 taskENTER_CRITICAL】。
 *      旧实现曾在算法侧用临界区，把 BASEPRI 提到优先级 5，而 SPI1/DMA1_Ch1/2
 *      的 NVIC 优先级也是 5，导致 DMA 完成中断被屏蔽、采集从 1 kHz 掉到 254 Hz。
 *
 * 栈：取值由【实测高水位】决定，不是拍脑袋。
 *   采集任务只承载 read() 的两段 DMA 等待与 TAG 解包，实测高水位仅约 89 词
 *   （512 词栈时 cap_stack_free = 423），取 256 词留近 3 倍余量；
 *   融合任务不再承担 read()，承载 ESKF 与快照发布，取 768 词
 *   （单任务版采集+融合合计高水位约 574 词，拆分后 read() 部分移出）。
 * 这两个数字必须在改动后重新核对 cap_stack_free / ins_stack_free 是否仍 >100 词。 */
constexpr uint16_t kCapTaskStackWords        = 256U;
constexpr uint16_t kInsTaskStackWords        = 768U;
constexpr uint16_t kPublisherTaskStackWords  = 384U;
constexpr uint32_t kCpuMhz = 170U;         /* DWT 周期 → 微秒换算 */

/* 板级绑定：片选由产品装配层决定（CubeMX 中 PA4 初始为高）。 */
/* 这两个常量放在 imu_capture.cpp，因为它们需要 CubeMX 的 GPIO 类型。 */

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
    cfg.fifo_max_words_per_read = kFifoMaxWordsPerRead;
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

    /* 静止检测低通的时间常数（原默认 0.5 s）。
     *
     * 2026-11-17 改为 0.15 s——修 "运动后静止，yaw 突变 0.45~1.6°"。
     *
     * 根因（yaw_full.csv 实测，i=7837，rest 0->1 那一拍）：
     *   bias_z    = +0.002395   （当前估计，接近真值 +0.002237）
     *   rest_lp_z = -0.001129   （低通后的静止参考，**符号都是错的**）
     *   残差 = rest_lp_z − bias_z = -0.003523 rad/s
     * 运动中 |gyro| 峰值 20 rad/s，而 τ=0.5 s 的二阶低通需要约 3~5τ
     * （1.5~2.5 s）才收敛；rest_min_t 只等 1.0 s（= 2τ），残余未衰减完，
     * 于是 _correct_rest 把这个残余当成零偏真值，把 bias 拉向错误方向，
     * yaw 以错误速率积分——表现为静止后 0.5 s 内 yaw 突变。
     *
     * 把 τ 降到 0.15 s 后，收敛时间缩到 0.45~0.75 s，1.0 s 的等待已足够
     * （约 6.7τ）。代价：低通更"薄"，静止判据对瞬时噪声更敏感；
     * 若实测静止判决变得难以建立（rest_t 反复清零），应放宽 rest_th_gyr
     * 或适当提高 rest_min_t。 */
    cfg.rest_filter_tau = 0.15f;
    /* 运动中是否允许倾斜观测修正零偏。
     *
     * 2026-11-17 关闭（true -> false），与 lsm6dsv_test/vqf_test.cpp 对齐——
     * 那个工程用 false，实测没有"静止后 yaw 突变"的问题。
     *
     * 作用点：pyro_algo_vqf.cpp:_gate() 的
     *     if (!motion_bias_est_enabled || rest_detected) { 零偏行 K 清零 }
     * true 时运动中保留零偏行，倾斜观测可通过交叉协方差 P01 改 bias_z；
     * false 时零偏只由静止观测（_correct_rest）修改。
     *
     * 实测代价很小：yaw_30s.csv 里运动 8 s，该通道只把 bias_z 改了
     * 0.000047 rad/s（0.0027 °/s），且运动段 acc_rej=1 占 62.9%，
     * 大部分时间根本执行不到。因此关闭它不会损失有效的零偏估计能力，
     * 但能排除"多一条 bias_z 修改通道"这个变量，便于定位 yaw 突变。 */
    cfg.motion_bias_est_enabled = false;
    cfg.rest_bias_est_enabled   = true;
    cfg.acc_update_period = 0.002f;
    return cfg;
}

} // namespace config
} // namespace imu_module

#endif /* IMU_MODULE_CONFIG_IMU_CONFIG_H */
