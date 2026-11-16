/**
 * @file    test_protocol.cpp
 * @brief   协议 v1 编解码的主机验证（不依赖 HAL/FreeRTOS）。
 *
 * 覆盖四类结论：
 *   ① PROTOCOL.md 里的字节测试向量必须逐字节一致（UART 帧 + 三帧 CAN）。
 *   ② 反向封包必须把"驱动解包出来的物理量"还原成原码：对 16 位全码值、8 个装配
 *      方向、多组量程与静态偏置做穷举，恢复误差恒为 0。这是 PROTOCOL.md 的验收判据。
 *   ③ 姿态无效时四元数四字必须为 0（否则接收方会把单位阵误认为"水平"）。
 *   ④ 取整/限幅边界：±0.5 远离零、越界饱和、NaN/Inf 归零。
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "imu_mount.h"
#include "imu_protocol.h"
#include "imu_snapshot.h"

using imu_module::imu_publish_params_t;
using imu_module::imu_snapshot_t;

namespace
{

int g_failures = 0;
int g_checks   = 0;

void check(bool cond, const char *what)
{
    ++g_checks;
    if (!cond)
    {
        ++g_failures;
        std::printf("  FAIL %s\n", what);
    }
}

/* 构造一份"驱动刚解包出来"的快照：给定原码，算出安装系物理量。
 * 正向链路 = 驱动解包（raw → 物理量，扣零偏）+ 装配映射（device → install）。 */
void build_snapshot_from_raw(imu_snapshot_t &snapshot, imu_publish_params_t &params,
                             imu_direct_t mount, float acc_range_g, float gyro_range_dps,
                             const float bias_dev[3], const int16_t acc_raw[3],
                             const int16_t gyro_raw[3])
{
    params.acc_mps2_per_lsb  = acc_range_g * 9.80665f / 32768.0f;
    params.gyro_rps_per_lsb  = (gyro_range_dps / 32768.0f) * 0.017453292519943295f;
    params.gyro_bias_dev[0]  = bias_dev[0];
    params.gyro_bias_dev[1]  = bias_dev[1];
    params.gyro_bias_dev[2]  = bias_dev[2];
    mount_inverse_indices(mount, params.inv_idx, params.inv_neg);

    /* 正向装配映射（device → install） */
    uint8_t fwd_idx[3];
    bool    fwd_neg[3];
    mount_indices(mount, fwd_idx, fwd_neg);

    snapshot = imu_snapshot_t{};
    snapshot.valid = true;
    snapshot.quat[0] = 1.0f;
    /* 正向定义是 install[j] = s[j] * device[p[j]]：符号属于【目标轴 j】。
     * 若写成 install[p[i]] = s[i]*device[i]（把符号挂在源轴上），换轴方向会算错——
     * 这正是 PROTOCOL.md 提醒"反向不能直接复用正向表"的原因。 */
    for (uint8_t j = 0U; j < 3U; ++j)
    {
        const uint8_t src = fwd_idx[j];
        const float acc_dev  = static_cast<float>(acc_raw[src]) * params.acc_mps2_per_lsb;
        const float gyro_dev = static_cast<float>(gyro_raw[src]) * params.gyro_rps_per_lsb -
                               bias_dev[src];
        snapshot.accel[j] = fwd_neg[j] ? -acc_dev : acc_dev;
        snapshot.gyro[j]  = fwd_neg[j] ? -gyro_dev : gyro_dev;
    }
    snapshot.params = params;
}

/* ① 文档字节向量 */
void test_documented_vector()
{
    std::printf("[1] PROTOCOL.md 字节向量\n");

    const float bias[3] = {0.0f, 0.0f, 0.0f};
    const int16_t acc_raw[3]  = {0, 0, 8192};
    const int16_t gyro_raw[3] = {1000, -2000, 3000};

    imu_snapshot_t snapshot{};
    imu_publish_params_t params{};
    /* ±4 g / ±2000 dps，DIRECT_1（恒等映射），零偏 0 */
    build_snapshot_from_raw(snapshot, params, imu_direct_t::DIRECT_1, 4.0f, 2000.0f, bias,
                            acc_raw, gyro_raw);

    const auto payload = imu_module::protocol::encode_payload(snapshot);

    /* 负载：q=1,0,0,0 → 7FFF,0,0,0；acc=0,0,8192；gyro=1000,-2000,3000 */
    const int16_t expect_payload[10] = {32767, 0, 0, 0, 0, 0, 8192, 1000, -2000, 3000};
    for (std::size_t i = 0U; i < 10U; ++i)
    {
        check(payload.word[i] == expect_payload[i], "payload word mismatch");
    }

    uint8_t frame[imu_module::protocol::kUartFrameSize] = {};
    const std::size_t n = imu_module::protocol::encode_uart_frame(payload, frame);
    check(n == 26U, "frame size");

    const uint8_t expect_frame[26] = {
        0xA5, 0x5A, 0x14, 0x00,                                     /* 帧头 + 长度 20 */
        0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             /* qw qx qy qz */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x20,                         /* acc x y z */
        0xE8, 0x03, 0x30, 0xF8, 0xB8, 0x0B,                         /* gyro x y z */
        0x87, 0xBD                                                  /* CRC16 低字节在前 */
    };
    for (std::size_t i = 0U; i < 26U; ++i)
    {
        if (frame[i] != expect_frame[i])
        {
            std::printf("  FAIL frame[%zu]=0x%02X expect 0x%02X\n", i, frame[i], expect_frame[i]);
            ++g_failures;
        }
    }
    ++g_checks;

    /* CRC 标准校验向量 */
    const uint8_t ascii[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    check(imu_module::protocol::crc16_ccitt_false(ascii, 9U) == 0x29B1U, "CRC check value");

    /* CAN 三帧：基址 0x300 + node*0x10 */
    imu_module::protocol::can_frame_t cans[imu_module::protocol::kCanFrameCount] = {};
    imu_module::protocol::encode_can_frames(1U, payload, cans);
    check(cans[0].id == 0x310U && cans[1].id == 0x311U && cans[2].id == 0x312U, "CAN ids");
    const uint8_t expect_can0[8] = {0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t expect_can1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00};
    const uint8_t expect_can2[8] = {0xE8, 0x03, 0x30, 0xF8, 0xB8, 0x0B, 0x00, 0x00};
    check(std::memcmp(cans[0].data, expect_can0, 8) == 0, "CAN frame 0");
    check(std::memcmp(cans[1].data, expect_can1, 8) == 0, "CAN frame 1");
    check(std::memcmp(cans[2].data, expect_can2, 8) == 0, "CAN frame 2");
}

/* ② 原码穷举回环：8 个方向 × 多组量程/偏置 × 全 16 位码值 */
void test_raw_round_trip()
{
    std::printf("[2] 原码穷举回环（8 方向 × 量程 × 偏置 × 全 16 位）\n");

    const float acc_ranges[]  = {2.0f, 4.0f, 8.0f, 16.0f};
    const float gyro_ranges[] = {125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f};

    const float biases[3][3] = {
        {0.0f, 0.0f, 0.0f},
        {0.010767111f, -0.013010208f, 0.0014672983f},
        {-0.05f, 0.07f, -0.09f}
    };
    const imu_direct_t mounts[8] = {
        imu_direct_t::DIRECT_1, imu_direct_t::DIRECT_2, imu_direct_t::DIRECT_3,
        imu_direct_t::DIRECT_4, imu_direct_t::DIRECT_5, imu_direct_t::DIRECT_6,
        imu_direct_t::DIRECT_7, imu_direct_t::DIRECT_8
    };

    uint32_t cases = 0U;
    int32_t  worst = 0;
    for (const auto mount : mounts)
    {
        for (const float acc_range : acc_ranges)
        {
            for (const float gyro_range : gyro_ranges)
            {
                for (const auto &bias : biases)
                {
                    /* 逐码值扫：每轴用同一码值同时验证三轴，等价于逐轴穷举 */
                    for (int32_t code = -32768; code <= 32767; ++code)
                    {
                        const int16_t raw[3] = { static_cast<int16_t>(code),
                                                 static_cast<int16_t>(code),
                                                 static_cast<int16_t>(code) };
                        imu_snapshot_t snapshot{};
                        imu_publish_params_t params{};
                        build_snapshot_from_raw(snapshot, params, mount, acc_range,
                                                gyro_range, bias, raw, raw);
                        const auto payload = imu_module::protocol::encode_payload(snapshot);
                        for (std::size_t axis = 0U; axis < 3U; ++axis)
                        {
                            const int32_t dacc =
                                static_cast<int32_t>(payload.word[4U + axis]) - code;
                            const int32_t dgyro =
                                static_cast<int32_t>(payload.word[7U + axis]) - code;
                            const int32_t diff = (dacc > 0 ? dacc : -dacc) +
                                                 (dgyro > 0 ? dgyro : -dgyro);
                            if (diff > worst) { worst = diff; }

                        }
                        ++cases;
                    }
                }
            }
        }
    }
    std::printf("    cases=%u  worst_error=%d\n", cases, worst);
    check(worst == 0, "raw round-trip must be exact");
}

/* ③ 无效姿态：四元数四字为零 */
void test_invalid_quaternion()
{
    std::printf("[3] 无效姿态\n");
    imu_snapshot_t snapshot{};
    snapshot.valid = false;
    snapshot.quat[0] = 1.0f; /* 内部是单位阵，但协议上必须报零 */
    snapshot.params.acc_mps2_per_lsb  = 4.0f * 9.80665f / 32768.0f;
    snapshot.params.gyro_rps_per_lsb  = (2000.0f / 32768.0f) * 0.017453292519943295f;
    snapshot.params.inv_idx[0] = 0U;
    snapshot.params.inv_idx[1] = 1U;
    snapshot.params.inv_idx[2] = 2U;

    const auto payload = imu_module::protocol::encode_payload(snapshot);
    for (std::size_t i = 0U; i < 4U; ++i)
    {
        check(payload.word[i] == 0, "invalid quat must be zero");
    }
}

/* ④ 取整与限幅边界 */
void test_round_clamp()
{
    std::printf("[4] 取整/限幅边界\n");
    using imu_module::protocol::round_clamp;
    check(round_clamp(0.4f, -32768, 32767) == 0, "0.4 -> 0");
    check(round_clamp(0.5f, -32768, 32767) == 1, "0.5 -> 1 (away from zero)");
    check(round_clamp(-0.5f, -32768, 32767) == -1, "-0.5 -> -1");
    check(round_clamp(1.5f, -32768, 32767) == 2, "1.5 -> 2");
    check(round_clamp(32767.4f, -32768, 32767) == 32767, "high clamp");
    check(round_clamp(-32768.6f, -32768, 32767) == -32768, "low clamp");
    check(round_clamp(1.0e9f, -32768, 32767) == 32767, "huge -> high");
    check(round_clamp(std::nanf(""), -32768, 32767) == 0, "NaN -> 0");
    check(round_clamp(INFINITY, -32768, 32767) == 0, "+Inf -> 0 (fault marker)");
    check(round_clamp(-INFINITY, -32768, 32767) == 0, "-Inf -> 0 (fault marker)");
}

} // namespace

int main()
{
    std::printf("protocol v1 test\n");
    test_documented_vector();
    test_raw_round_trip();
    test_invalid_quaternion();
    test_round_clamp();
    std::printf("%s: %d checks, %d failures\n", g_failures == 0 ? "PASS" : "FAILED",
                g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
