#ifndef IMU_MOUNT_H
#define IMU_MOUNT_H

#include <cmath>
#include <cstdint>

/* ============================================================================
 * 安装方向：板级装配参数（器件坐标系 → 安装坐标系）
 *
 * 归属产品装配层。器件驱动不感知本文件——驱动的输出契约是"器件坐标系的
 * 物理量（单位换算 + 器件零偏已扣除）"，不感知产品如何装配。
 *
 * 为什么映射可以放在输出端：8 种方向均为符号置换矩阵（行列式全为 +1），
 * 而 VQF 内部运算对它不变——逐通道 Butterworth 低通、逐轴零偏限幅、
 * 模长 / NIS 判据都只是换轴与改符号。因此"驱动端逐样本 remap"与"输出端
 * 逐发布 remap"数学等价，只是浮点舍入路径不同。
 *
 * 硬约束：该等价性只在符号置换下成立。若将来支持任意安装角（一般旋转，或
 * 非正交的安装标定矩阵），逐通道低通不再与旋转可交换、逐轴限幅不再旋转
 * 不变，两种放法结果不同，必须重新评估算法输入坐标系，而不是扩展本文件。
 *
 * 反向映射（安装系 → 器件系，即 Mᵀ）目前无人使用。将来通信层做反向封包时
 * 【不能直接复用正向 (idx,neg)】：正向 out[j] = s[j]·in[p[j]]，反向是
 * out[i] = s[p⁻¹(i)]·in[p⁻¹(i)]，负号跟着 p⁻¹ 走。DIRECT_2 / DIRECT_4 这类
 * "换轴 + 非对称取反"的方向照搬会算错，须由 mount_indices 导出逆表。
 * ========================================================================== */

enum class imu_direct_t : uint8_t {
    NONE     = 0,
    DIRECT_1 = 1,
    DIRECT_2 = 2,
    DIRECT_3 = 3,
    DIRECT_4 = 4,
    DIRECT_5 = 5,
    DIRECT_6 = 6,
    DIRECT_7 = 7,
    DIRECT_8 = 8
};

/* 产品安装方向：换装配方向只改这一行。 */
constexpr imu_direct_t k_imu_direct = imu_direct_t::DIRECT_1;

/* 正向映射参数（器件系 → 安装系）：
 *   安装系[i] = neg[i] ? -器件系[idx[i]] : +器件系[idx[i]]
 * 与 PYRo INS 的 DIRECT_1..8 约定一致。本表在工程内唯一。 */
inline void mount_indices(imu_direct_t d, uint8_t idx[3], bool neg[3])
{
    idx[0] = 0U; idx[1] = 1U; idx[2] = 2U;
    neg[0] = false; neg[1] = false; neg[2] = false;

    switch (d) {
        case imu_direct_t::DIRECT_2: idx[0] = 1U; idx[1] = 0U; neg[1] = true; break;
        case imu_direct_t::DIRECT_3: neg[0] = true; neg[1] = true; break;
        case imu_direct_t::DIRECT_4: idx[0] = 1U; idx[1] = 0U; neg[0] = true; break;
        case imu_direct_t::DIRECT_5: neg[1] = true; neg[2] = true; break;
        case imu_direct_t::DIRECT_6: idx[0] = 1U; idx[1] = 0U; neg[0] = true; neg[1] = true; neg[2] = true; break;
        case imu_direct_t::DIRECT_7: neg[0] = true; neg[2] = true; break;
        case imu_direct_t::DIRECT_8: idx[0] = 1U; idx[1] = 0U; neg[2] = true; break;
        case imu_direct_t::DIRECT_1:
        case imu_direct_t::NONE:
        default: break; /* 单位映射 */
    }
}

/* 向量换系（热路径）：就地转成安装系。 */
inline void mount_apply(float v[3], imu_direct_t d)
{
    uint8_t idx[3];
    bool    neg[3];
    mount_indices(d, idx, neg);
    const float t[3] = { v[idx[0]], v[idx[1]], v[idx[2]] }; /* 先取出，避免原地覆盖 */
    v[0] = neg[0] ? -t[0] : t[0];
    v[1] = neg[1] ? -t[1] : t[1];
    v[2] = neg[2] ? -t[2] : t[2];
}

/* 3x3 旋转矩阵 → 单位四元数 [w,x,y,z]（Shepperd 分支法，取最大分量避免除零）。
 * 仅由 mount_quat_inverse 在初始化时调用一次。 */
inline void rotation_to_quat(const float R[3][3], float q[4])
{
    float d[4];
    d[0] = 1.0f + R[0][0] + R[1][1] + R[2][2];
    d[1] = 1.0f + R[0][0] - R[1][1] - R[2][2];
    d[2] = 1.0f - R[0][0] + R[1][1] - R[2][2];
    d[3] = 1.0f - R[0][0] - R[1][1] + R[2][2];
    uint8_t k = 0U;
    for (uint8_t i = 1U; i < 4U; ++i) { if (d[i] > d[k]) { k = i; } }

    const float s = 0.5f / std::sqrt(d[k]); /* 1 / (4·最大分量) */
    switch (k) {
        case 0U: q[0] = 0.25f / s;           q[1] = (R[2][1] - R[1][2]) * s; q[2] = (R[0][2] - R[2][0]) * s; q[3] = (R[1][0] - R[0][1]) * s; break;
        case 1U: q[0] = (R[2][1] - R[1][2]) * s; q[1] = 0.25f / s;           q[2] = (R[0][1] + R[1][0]) * s; q[3] = (R[0][2] + R[2][0]) * s; break;
        case 2U: q[0] = (R[0][2] - R[2][0]) * s; q[1] = (R[0][1] + R[1][0]) * s; q[2] = 0.25f / s;           q[3] = (R[1][2] + R[2][1]) * s; break;
        default: q[0] = (R[1][0] - R[0][1]) * s; q[1] = (R[0][2] + R[2][0]) * s; q[2] = (R[1][2] + R[2][1]) * s; q[3] = 0.25f / s;           break;
    }
}

/* 姿态换系参数：算出 m 的逆四元数。由同一张正向表构造，保证与向量换系同源。
 * 初始化时调用一次并缓存，热路径只做 quat_to_install 的 16 次乘法。 */
inline void mount_quat_inverse(imu_direct_t d, float m_inv[4])
{
    uint8_t idx[3];
    bool    neg[3];
    mount_indices(d, idx, neg);

    float R[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    for (uint8_t i = 0U; i < 3U; ++i) {
        R[i][idx[i]] = neg[i] ? -1.0f : 1.0f; /* 安装系[i] = s[i]·器件系[idx[i]] */
    }

    float m[4];
    rotation_to_quat(R, m);
    m_inv[0] =  m[0]; /* 单位四元数的逆即共轭 */
    m_inv[1] = -m[1];
    m_inv[2] = -m[2];
    m_inv[3] = -m[3];
}

/* 姿态换系（热路径）：器件系→世界系 转成 安装系→世界系，q_inst = q_dev ⊗ m⁻¹。
 * 推导：v_w = q_dev ⊗ v_dev ⊗ q_dev*，而 v_dev = m⁻¹ ⊗ v_inst ⊗ m，
 * 故 v_w = (q_dev ⊗ m⁻¹) ⊗ v_inst ⊗ (q_dev ⊗ m⁻¹)*，右乘而非左乘。 */
inline void quat_to_install(float q[4], const float m_inv[4])
{
    const float aw = q[0], ax = q[1], ay = q[2], az = q[3];
    const float bw = m_inv[0], bx = m_inv[1], by = m_inv[2], bz = m_inv[3];
    q[0] = aw * bw - ax * bx - ay * by - az * bz;
    q[1] = aw * bx + ax * bw + ay * bz - az * by;
    q[2] = aw * by - ax * bz + ay * bw + az * bx;
    q[3] = aw * bz + ax * by - ay * bx + az * bw;
}

#endif /* IMU_MOUNT_H */
