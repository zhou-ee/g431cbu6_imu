/**
 * @file    imu_protocol.cpp
 * @brief   协议 v1 编解码实现。纯函数：无状态、无硬件依赖，可在主机上逐字节验证。
 *
 * 数值约定（PROTOCOL.md）：
 *   qw..qz = round(q × 32767)，范围 [-32767, 32767]，无效姿态四字全零
 *   acc_raw = round((Mᵀ · acc_installed) / Sa)，限幅 [-32768, 32767]
 *   gyro_raw = round((Mᵀ · gyro_installed + b) / Sg)，限幅 [-32768, 32767]
 * 取整一律"就近、中点远离零"，与驱动正向解包使用相同的常量关系。
 */

#include "imu_protocol.h"

#include <cmath>

namespace imu_module
{
namespace protocol
{

int32_t round_clamp(float value, int32_t low, int32_t high)
{
    if (!std::isfinite(value))
    {
        return 0; /* 兜底：调用方本应先判有限性 */
    }
    /* 先限幅再取整：避免 lroundf 在大值上的未定义行为，也保证输出不越界 */
    if (value >= static_cast<float>(high))
    {
        return high;
    }
    if (value <= static_cast<float>(low))
    {
        return low;
    }
    return static_cast<int32_t>(std::lroundf(value)); /* 半分远离零 */
}

uint16_t crc16_ccitt_false(const uint8_t *data, std::size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0U; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8U);
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

uint32_t can_base_id(uint8_t node_id)
{
    const uint8_t node = static_cast<uint8_t>(node_id & kNodeIdMax);
    return kCanBaseId + static_cast<uint32_t>(node) * kCanIdStep;
}

payload_t encode_payload(const imu_snapshot_t &snapshot)
{
    payload_t payload{};
    const imu_publish_params_t &pm = snapshot.params;

    /* ① 四元数：无效姿态输出零四元数（接收端用零范数识别），
     *    不可直接封包——此时模块内部姿态是单位阵，会被误认为水平。 */
    if (snapshot.valid)
    {
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            payload.word[i] = static_cast<int16_t>(
                round_clamp(snapshot.quat[i] * kQuatScale, -32767, 32767));
        }
    }

    /* ② 加速度：安装系 → 器件系（Mᵀ）→ 除以 Sa */
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        const float installed = snapshot.accel[pm.inv_idx[i]];
        const float device    = pm.inv_neg[i] ? -installed : installed;
        payload.word[4U + i] = static_cast<int16_t>(
            round_clamp(device / pm.acc_mps2_per_lsb, -32768, 32767));
    }

    /* ③ 角速度：逆装配方向 → 加回驱动静态偏置 b → 除以 Sg */
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        const float installed = snapshot.gyro[pm.inv_idx[i]];
        const float device    = (pm.inv_neg[i] ? -installed : installed) +
                                pm.gyro_bias_dev[i];
        payload.word[7U + i] = static_cast<int16_t>(
            round_clamp(device / pm.gyro_rps_per_lsb, -32768, 32767));
    }

    return payload;
}

std::size_t encode_uart_frame(const payload_t &payload, uint8_t out[kUartFrameSize])
{
    out[0] = kUartSync0;
    out[1] = kUartSync1;
    out[2] = static_cast<uint8_t>(kPayloadSize & 0xFFU);        /* 0x14 */
    out[3] = static_cast<uint8_t>((kPayloadSize >> 8U) & 0xFFU); /* 0x00 */
    for (std::size_t i = 0U; i < 10U; ++i)
    {
        const uint16_t value = static_cast<uint16_t>(payload.word[i]);
        out[4U + 2U * i]      = static_cast<uint8_t>(value & 0xFFU);
        out[4U + 2U * i + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    }
    /* CRC 覆盖帧头与长度：接收端必须先按帧头对齐再对整段前缀校验，
     * 长度也在校验范围内，长度被篡改时校验必然失败。 */
    const uint16_t crc = crc16_ccitt_false(out, kUartFrameSize - 2U);
    out[kUartFrameSize - 2U] = static_cast<uint8_t>(crc & 0xFFU);
    out[kUartFrameSize - 1U] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
    return kUartFrameSize;
}

void encode_can_frames(uint8_t node_id, const payload_t &payload,
                       can_frame_t out[kCanFrameCount])
{
    const uint32_t base = can_base_id(node_id);

    /* 帧 0：完整四元数 qw qx qy qz */
    out[0].id = base + 0U;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        const uint16_t value = static_cast<uint16_t>(payload.word[i]);
        out[0].data[2U * i]      = static_cast<uint8_t>(value & 0xFFU);
        out[0].data[2U * i + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    }

    /* 帧 1：加速度三轴 + 保留；帧 2：角速度三轴 + 保留 */
    for (uint8_t frame = 0U; frame < 2U; ++frame)
    {
        out[frame + 1U].id = base + frame + 1U;
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            const uint16_t value =
                static_cast<uint16_t>(payload.word[4U + 3U * frame + axis]);
            out[frame + 1U].data[2U * axis]      = static_cast<uint8_t>(value & 0xFFU);
            out[frame + 1U].data[2U * axis + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
        }
        out[frame + 1U].data[6] = 0U; /* 保留字段 */
        out[frame + 1U].data[7] = 0U;
    }
}

} // namespace protocol
} // namespace imu_module
