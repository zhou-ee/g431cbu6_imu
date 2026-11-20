/**
 * @file    imu_protocol.h
 * @brief   协议 v1 编解码（纯函数，零硬件依赖）。
 *
 * 唯一规格来源是 Module/PROTOCOL.md；本文件不引用 HAL、FreeRTOS 或任何驱动，
 * 因此可以整段拿到主机上做逐字节验证。
 *
 * 编码方向的三条规则（对应 PROTOCOL.md）：
 *   ① 四元数按 32767 倍就近取整，范围 [-32767,32767]；姿态无效时四字全零。
 *   ② 加速度：安装系 → 器件系（Mᵀ）→ 除以 Sa → 就近取整 → 限幅 [-32768,32767]。
 *   ③ 角速度：逆装配方向 → 加回驱动静态偏置 b → 除以 Sg → 就近取整 → 限幅。
 */

#ifndef IMU_MODULE_COMMUNICATION_IMU_PROTOCOL_H
#define IMU_MODULE_COMMUNICATION_IMU_PROTOCOL_H

#include <cstddef>
#include <cstdint>

#include "imu_fusion.h"   /* 快照类型随融合模块 */

namespace imu_module
{
namespace protocol
{

/* ------------------------------ 常量 ------------------------------ */
constexpr std::size_t kPayloadSize   = 20U;  /* 十个 int16 测量字 */
constexpr std::size_t kUartFrameSize = 26U;  /* 帧头 2 + 长度 2 + 负载 20 + CRC 2 */
constexpr uint8_t     kUartSync0     = 0xA5U;
constexpr uint8_t     kUartSync1     = 0x5AU;
constexpr std::size_t kCanFrameCount = 3U;
constexpr uint32_t    kCanBaseId     = 0x300U;
constexpr uint32_t    kCanIdStep     = 0x10U;
constexpr uint8_t     kNodeIdMax     = 31U;
constexpr float       kQuatScale     = 32767.0f;

/* ============================ 模块量程（协议自定） ============================
 * 这是【本模块对外的编码约定】，不是器件参数：
 *   满码 ±32768  ⇔  ±kModuleAccRangeG / ±kModuleGyroRangeDps
 * 收发两端按同一组常量换算，协议因此自包含——不 include 产品配置、
 * 也不依赖器件实际灵敏度。
 *
 * 注意：器件陀螺真实灵敏度是 0.070 dps/LSB（满码约 ±2294 dps），
 * 而本约定是 2000/32768 = 0.0610 dps/LSB，满码恰好对应 ±2000 dps。
 * 即模块标称量程用满整个 int16，器件侧 14.7% 的余量不对外暴露，
 * 超过 ±2000 dps 的真实角速度会被削顶——这是标称量程的预期行为。
 *
 * 换量程时只改这里两行，与 imu_config.h 的器件量程各自独立维护。 */
constexpr uint16_t kModuleAccRangeG    = 4U;      /* ±g */
constexpr uint16_t kModuleGyroRangeDps = 2000U;   /* ±°/s */

/* 编码系数由上面两行线性推出，不另立换算表。 */
constexpr float kAccMps2PerLsb =
    static_cast<float>(kModuleAccRangeG) * 9.80665f / 32768.0f;
/* 陀螺的余量正是这 87.19% 的线性约定，×π/180 只是 deg→rad，无其他修正。 */
constexpr float kGyroRpsPerLsb =
    static_cast<float>(kModuleGyroRangeDps) / 32768.0f * 0.0174532925199433f;

/** 公共测量负载：十个 int16，索引顺序即线上偏移顺序（小端）。 */
struct payload_t
{
    int16_t word[10];
};

/** 一帧经典 CAN（标准 11 位 ID，固定 8 字节）。 */
struct can_frame_t
{
    uint32_t id;
    uint8_t  data[8];
};

/* ------------------------------ 编码 ------------------------------ */

/** 由快照生成负载。
 *  方向表取自快照本身（`imu_publish_params_t`），因此装配方向切换时
 *  不会出现"新方向换算旧样本"的错配；量程系数是本文件的编译期常量。 */
payload_t encode_payload(const imu_snapshot_t &snapshot);

/** 生成完整 UART 帧。CRC 覆盖帧内 0..23 字节（含帧头与长度字段）。
 *  @return 写入字节数（恒为 kUartFrameSize） */
std::size_t encode_uart_frame(const payload_t &payload, uint8_t out[kUartFrameSize]);

/** 生成一组三帧 CAN：`base_id+0/1/2`，第二、三帧末两字节为保留（填 0）。 */
void encode_can_frames(uint8_t node_id, const payload_t &payload,
                       can_frame_t out[kCanFrameCount]);

/** 该节点的一组 CAN 基址：`0x300 + node_id * 0x10`。 */
uint32_t can_base_id(uint8_t node_id);

/* ------------------------------ 工具 ------------------------------ */

/** UART 帧 CRC 由 PYRo/Algorithm/CRC 提供（pyro_crc.h）：
 *  `append_crc16_check_sum(frame, 26)` 计算前 24 字节并把结果写入末尾两字节，
 *  低字节在前。**本层不再自带 CRC 实现**，算法参数以 PYRo 中的定义为准。
 *  该实现属反射查表族（poly 0x1021 反射、init 0xFFFF、无 xorout），
 *  与 CRC-16/CCITT-FALSE 不兼容；对端正式发布协议时须按此参数对接。 */

/** 就近取整、中点远离零，并限幅到 [low, high]。
 *  非有限值（NaN/±Inf）返回 0：它们是上游故障信号，不应伪装成合法的饱和码值，
 *  接收侧看到 0 也不会误判为"满量程"。 */
int32_t round_clamp(float value, int32_t low, int32_t high);

} // namespace protocol
} // namespace imu_module

#endif /* IMU_MODULE_COMMUNICATION_IMU_PROTOCOL_H */
