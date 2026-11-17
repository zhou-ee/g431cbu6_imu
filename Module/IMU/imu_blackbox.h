/**
 * @file    imu_blackbox.h
 * @brief   临时诊断黑匣子：把 yaw 突变归因到"哪一路修正"上。
 *
 * ============================ 现象与候选因 ============================
 * 现象：剧烈运动后静止约 0.5 s，yaw 突变。
 *
 * 代码上的因果链（pyro_algo_vqf.cpp）：
 *   ① 剧烈运动期间加速度多被门控拒绝，rest_t 恒 0、rest_detected = false。
 *      此时 _gate 里 `!motion_bias_est_enabled || rest_detected` 为假
 *      （产品配置 motion_bias_est_enabled = true），零偏行【保留】，
 *      运动中的倾斜观测仍会修正 bias。
 *   ② 静止满 rest_min_t（产品 1.0 s）后 rest_detected 翻 true，
 *      立刻走 _correct_rest()：H = [0|I] 直接观测三轴零偏，
 *      残差 = rest_gyr_lp - bias，把 bias 一次性拉向静止低通值。
 *   ③ yaw 全靠陀螺积分 -> z 零偏突变 = yaw 速率突变 = 观感上的"yaw 突变"。
 *
 * 这条链是【推断】，必须实测每次修正对 yaw 的贡献才能定案。
 *
 * ============================ 记录策略 ============================
 * RAM 只剩约 9 KB（_end 0x20005BE8 -> _estack 0x20008000），
 * 1 kHz 全速记 20 B/行 只够 0.4 s，抓不住"运动->静止"全过程。
 *
 * 采用【200 Hz 降采样 + 事件加记】：
 *   - 常规：每 5 拍记一行（200 Hz），256 行覆盖 1.28 s；
 *     yaw 突变是百毫秒级现象，5 ms 分辨率足够。
 *   - 事件：rest 跳变、|dyaw| 超阈值时【额外】立刻记一行，
 *     保证跳变瞬间不被采样节拍漏掉。
 *   - 环形覆盖最旧，任何时刻都保有最近 1.28 s。
 *
 * 抓取流程：复位 -> 剧烈运动 -> 静止 -> 等 2 s -> 回读。
 */

#ifndef IMU_BLACKBOX_H
#define IMU_BLACKBOX_H

#include <cstdint>

namespace imu_module
{
namespace blackbox
{

/* 一行 20 B。gdb 按地址连续 dump 后按此顺序解析。
 * 刻意只用 uint16/uint8/float，保证无填充、可整块 memcpy 风格 dump。 */
struct row_t
{
    uint16_t tick_mod;  /* +0  tick 低 16 位（1 kHz，65.5 s 回绕一次，够用） */
    uint8_t  rest;      /* +2  rest_detected */
    uint8_t  flags;     /* +3  bit0=rest 跳变 bit1=|dyaw| 超阈值 bit2=常规采样 */
    float    yaw;       /* +4  安装系、已归零 yaw，° */
    float    bias_z;    /* +8  z 轴零偏估计，rad/s —— yaw 漂移直接来源 */
    float    dyaw;      /* +12 相对上一行的 yaw 变化，° */
    float    rest_t;    /* +16 连续静止计时，s */
};

/* 256 行 × 20 B = 5120 B，留足余量给主栈与中断 */
constexpr uint16_t kRows      = 256U;
constexpr uint16_t kRowMask   = kRows - 1U;
constexpr uint8_t  kSampleDiv = 5U;        /* 每 5 拍一行 -> 200 Hz */
constexpr float    kDyawAlarm = 0.30f;     /* |dyaw| 超此值判为"突变"并加记，° */
constexpr uint8_t  kArmFreeze = 1U;        /* 1=检测到突变后冻结，保留现场 */

} // namespace blackbox
} // namespace imu_module

/* ============================ 调试观察量 ============================
 * 定义在 ins_service.cpp。gdb 直接按名字 watch，零依赖。
 * 注意：缓冲区用固定的链接期预留段，不占 BSS（见 _blackbox_ram）。 */
extern "C" {
extern volatile float    imu_bb_rows[5120 / 4]; /* 原始缓冲，按 blackbox::row_t 解析 */
extern volatile uint32_t imu_bb_head;           /* 下一行写入位置（0..255） */
extern volatile uint32_t imu_bb_total;          /* 累计写入行数 */
extern volatile uint32_t imu_bb_events;         /* 事件加记次数 */
extern volatile uint32_t imu_bb_div;            /* 采样分频计数器 */
extern volatile float    imu_bb_max_dyaw;       /* 全程 |dyaw| 最大值，° */
extern volatile float    imu_bb_last_yaw;       /* 上一拍 yaw，° */
extern volatile uint32_t imu_bb_rest_prev;      /* 上一拍 rest，用于检测跳变 */
extern volatile uint32_t imu_bb_frozen;         /* 1=已冻结，缓冲保留突变现场 */
extern volatile uint32_t imu_bb_enable;         /* 0=停止记录 */
}

#endif
