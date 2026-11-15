# 六轴 VQF

`pyro::vqf_t` 提供陀螺积分、加速度倾斜校正、静止检测和零偏估计。

## 公开接口

| 接口 | 用途 |
|---|---|
| `configure(config)` | 首次初始化，以及运行中统一修改配置 |
| `reset(bias, sigma)` | 重置全部状态，可指定初始零偏及标准差 |
| `update(data)` | 接收 `const imu_data_t &`，按时间戳合并处理两路新样本 |
| `get_state()` | 取得姿态、零偏、消费帧数、静止参考量和 KF 诊断的值快照 |

配置由 `vqf_t::config_t` 保存，包含六轴融合参数。更新间隔由输入时间戳决定。
成员注释标明单位。结果由 `vqf_t::state_t` 保存，内部低通与积分状态由实例管理。
公开方法统一为上表四项。静止调试使用快照中的 `rest_t / rest_gyr_lp / rest_acc_lp`。

## 基本使用

```cpp
#include "imu_base_drv.h"
#include "pyro_algo_vqf.h"

pyro::vqf_t attitude;
pyro::vqf_t::config_t config;

pyro::status_t attitude_init()
{
    config.tau_acc = 3.0f;
    return attitude.configure(config);
}

pyro::status_t attitude_update(const imu_data_t &data)
{
    return attitude.update(data);
}

// 控制循环读取一次快照。
// const auto state = attitude.get_state();
// state.quat / state.bias / state.rest_detected
```

`imu_data_t` 及两路样本类型使用本工程 `Component/IMU/imu_base_drv.h` 中的定义。
`gyro_sample_t::Gyro` 使用 rad/s，`acc_sample_t::Accel` 使用 m/s²，两者采用相同的右手坐标系。
输出 `quat` 按 `[w,x,y,z]` 存储，将传感器系向量旋转到 Z 向上的六轴参考系。
航向由初始条件确定。零偏和协方差分别使用 rad/s 与 (rad/s)²。
`bias_sigma` 为零偏标准差上界，单位 rad/s。

## 时间戳合并与异频批次

```cpp
attitude.configure(config);

// 驱动成功取到新数据后，直接交给 VQF。
imu.update();
const auto status = attitude.update(imu.read_imu_data());
const auto state = attitude.get_state();
imu.assume_gyro_point(state.consumed_gyr);
imu.assume_acc_point(state.consumed_acc);
// 根据 status 使用 state，剩余样本保留在两路流中。
(void)status;
```

VQF 从 `gyro.count / acc.count` 读取有效帧数，
用 `gyro.at(i) / acc.at(i)` 取样本（i=0 最旧，内部处理环形回绕），
两个游标按 timestamp 线性合并。
每次处理时间较早的样本；时间戳相同时先处理角速度，再处理加速度。
支持两路独立采样率、不同帧数、单路数据和空批次。

两路各自按采样时间排列，使用相同的器件时间基准，各批按采样时间先后提交。
比较采用 uint32 模运算，支持计数器回绕，本批时间跨度应小于 2^31 tick。
每路独立保存上一帧时间戳，跨批次延续，更新周期为
`dt = uint32_t(timestamp - previous_timestamp) * data.timestamp_tick_s`。
陀螺积分、所有低通、静止计时、零偏过程噪声与测量权重均使用对应的实际周期。
各路首帧初始化低通并建立时间基准，该帧的积分/计时为零。
同一路重复时间戳的帧消费后跳过，批次返回 `PYRO_WARNING`。
倒退时间戳或间隔达到 `2^31` tick 的帧返回 `PYRO_PARAM_ERROR`，保留该帧供处理。
设备重启或时间戳计数器被清零后调用 `reset()` 建立新的时间基准。

`imu_data_t::timestamp_tick_s` 是每个 tick 的秒数，通用默认值为 `1e-6`。
本工程 LSM6DSV16X 驱动根据数据手册 DS13510 第 9.52 节，在初始化时读取
有符号 `INTERNAL_FREQ_FINE`，设置
`timestamp_tick_s = 1 / (46080 * (1 + 0.0013 * FREQ_FINE))`。
其他数据源须填写自身时间戳单位。批次必须携带逐帧采样时间戳。
运行中修改 `timestamp_tick_s` 返回 `PYRO_PARAM_ERROR`，更换时间基准前调用 `reset()`。
原 VQF 配置项 `gyr_freq / acc_freq` 已移除，器件 ODR 配置继续由 IMU 驱动管理。
VQF 通过 `state.consumed_gyr / state.consumed_acc` 报告最近一次 update 的实际消费前缀。
调用方按这两个计数归还样本，避免重复处理已提交帧或移除尚未处理的尾部。

## 参数修改与重置

修改调用方持有的 config，再调用 `configure(config)`。
参数更新保留当前姿态和零偏，降低 `bias_clip` 会立即对已有零偏限幅。
`tau_acc` 变化保留低通输出和物理导数，实际更新周期变化时低通状态连续。
静止判据变化重启静止检测，运动零偏开关变化重启相应观测低通。
`bias_clip` 同时参与陀螺静止判据，修改时也会重启静止检测。
`bias_sigma_init` 更新初始方差及过程噪声增长的参考上限，调用 `reset()` 时重新设置先验协方差。

`reset()` 恢复单位四元数、零偏零值及配置的初始不确定度。
`reset(bias, sigma)` 可提供初始零偏，sigma=0 使用配置的初始不确定度。
输入非法配置或重置参数时，当前状态保留。

初始化前更新/重置返回 `PYRO_NOT_FOUND`。
非空流要求缓冲非空、`count <= capacity` 且 `head < capacity`，否则返回 `PYRO_PARAM_ERROR`。
非有限样本、非法 tick 单位或倒退时间戳同样返回 `PYRO_PARAM_ERROR`。
已初始化时，空批次返回 `PYRO_OK`，帧数为零的流允许传入空指针。
零加速度或低通向量无法确定方向时，该帧跳过校正，批次继续处理并最终返回 `PYRO_WARNING`。
零加速度会中断连续静止计时。静止确认需要陀螺仪与加速度计两路判据均满足。
其他错误结束批次，此前完成的样本及步骤保留。每次 update 开始时消费计数清零。
非法输入帧在提交前被拒绝，消费前缀停在该帧之前，调用方可修复输入后继续。
若加速度倾斜校正已经提交而后续 KF 失败，该加速度帧计入消费量，以避免重试时重复校正。
WARNING 帧同样计入消费量，批次继续处理后续样本。
通过 `state.initialized` 和 `state.bias_diagnostics` 读取初始化状态及最近 KF 操作结果。
同一实例的配置、更新与读取由调用方同步。

## 算法依赖

零偏估计使用现有 `kf_t<3,3>`，沿用 Cholesky 求解和 Joseph 协方差更新。
矩阵使用 PYRo Matrix，四元数运算使用 CMSIS-DSP。
`Algorithm/Butterworth` 独立提供 `configure / reset / process` 三个入口，
低通使用二阶连续 Butterworth 的状态转移，区间内输入按当前样本保持。
可变周期调用 `process(input, output, dt)`，支持零间隔初始化及长间隔。
启动阶段按实际 dt 计算时间加权均值，之后使用精确状态转移，周期相同时复用系数。
巴特沃斯模块统一为 `configure(tau) / reset() / process(input, output, dt)`，逐次显式提供周期。
低通系数和递推状态采用 double，输入输出采用 float。
所有模块使用固定存储。VQF 算法版权及 MIT 许可保留在本目录。

## 联调检查

demo 通过 `vqf_test_imu_bus_err / vqf_test_imu_data_err` 观察总线失败、FIFO 溢出和样本缓冲覆盖，
通过 `vqf_test_update_us_max` 观察解算耗时。未及时消费时，24 帧环形缓冲会覆盖最旧样本并计数。
时间戳可以提供丢帧区间的时长，区间运动仍按当前角速度样本保持来近似。
异频批次应按共同时间轴提交，已经提交较新陀螺帧后到达的旧加速度帧缺少历史姿态补偿。
可变步长低通在 dt 变化时重新计算系数，高采样率实时性须通过实际硬件测量。

IMU 标定使用传感器原坐标系的未扣偏置样本，结束后恢复安装方向。
标定失败恢复原偏置和安装方向。标定期间会清除并消费两路待处理样本。
