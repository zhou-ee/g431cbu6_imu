/**
 * @file    imu_fusion.h
 * @brief   融合侧对外契约：统一快照 + fuse_once/snapshot。
 *
 * ============================ 为什么快照在这里 ============================
 * 快照是"融合结果"的产物，与 seqlock 发布机制天然内聚，因此归入本模块，
 * 不再单独占用一个头文件。Communication 层只需 include 本文件即可拿到
 * 全部输出，且零 RTOS / 零驱动依赖。
 *
 * ============================ 一致性语义 ============================
 * 参考实现（Wheel_leg 的 13 个语义化 getter）各自读私有缓存，两次调用之间
 * 任务可能已翻拍，跨字段不同拍；协议要求"四元数 + 加速度 + 角速度同组"时
 * 那就是错数据。本模块只给一个【全量快照】，跨字段天然同拍。
 */

#ifndef IMU_MODULE_IMU_IMU_FUSION_H
#define IMU_MODULE_IMU_IMU_FUSION_H

#include <cstdint>

#include "pyro_core_def.h"

namespace imu_module
{

/** 反向封包所需的【装配方向】参数。
 *  与测量值在【同一次快照】里取出，保证换系用的是产生这些物理量的那组方向表。
 *
 *  不含量程/灵敏度/零偏：编码系数由协议层按模块量程自行定义（协议自包含），
 *  零偏已在快照的 gyro 里扣除，因此这里只剩与器件无关的几何信息。 */
struct imu_publish_params_t
{
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

namespace fusion
{

/** 算法配置 + 装配参数导出。需先完成 capture::init()（要读解包参数与零偏）。 */
pyro::status_t init();

/** 融合一拍：消费 SPSC 环形缓冲 → update → 归一化 → 归零 → 快照。
 *  @retval true  本次产出了新快照（调用方据此唤醒发布任务）
 *  @retval false 没有新测量或有效性变化，快照未更新 */
bool fuse_once();

/** 取一次完整快照（seqlock，跨字段同拍）。
 *  任意【任务】上下文可调用；**不可在 ISR 调用**——读侧依赖"写方一定能拿到
 *  CPU 把序号置回偶数"，写方在挂起调度器的窗口内发布，任务级读者不会空转。 */
imu_snapshot_t snapshot();

} // namespace fusion
} // namespace imu_module

/* ============================ 调试观察量 ============================
 * 定义在 imu_fusion.cpp。全局作用域 + extern "C"，保证 gdb 按裸名字可查。 */
extern "C" {
extern volatile int32_t  imu_service_vqf_ready;       /* 1=算法配置成功 */
extern volatile uint32_t imu_service_algo_rounds;     /* 融合轮数 */
extern volatile uint32_t imu_service_vqf_consumed;    /* 累计消费样本数（两路） */
extern volatile uint32_t imu_service_vqf_err;         /* 算法 ERROR 次数 */
extern volatile uint32_t imu_service_vqf_resets;      /* 融合恢复次数 */
extern volatile uint32_t imu_service_reset_reason;    /* 1 丢样 / 2 陈旧 / 3 倒退 / 4 长间隔 / 5 算法错误 */
extern volatile uint32_t imu_service_invalid_frames;  /* 生成的无效姿态快照数 */
extern volatile uint32_t imu_service_snapshot_valid;
extern volatile uint32_t imu_service_vqf_warn;        /* 算法 WARNING 次数 */
extern volatile int32_t  imu_service_vqf_status;      /* 最近一次 update 返回值 */
extern volatile uint32_t imu_service_vqf_us_max;      /* 单批融合峰值耗时，µs */
extern volatile uint32_t imu_service_vqf_us_avg;      /* 单批融合平均耗时，µs */
extern volatile uint32_t imu_service_loop_us_avg;     /* 融合整拍平均耗时，µs */
extern volatile uint32_t imu_service_ins_stack_free;  /* 融合任务栈余量（词） */

extern volatile float    imu_service_quat[4];         /* 安装系 [w,x,y,z] */
extern volatile float    imu_service_quat_norm;       /* 应恒 ≈ 1.0 */
extern volatile float    imu_service_roll;            /* ° */
extern volatile float    imu_service_pitch;           /* ° */
extern volatile float    imu_service_yaw;             /* °（相对产品零点） */
extern volatile float    imu_service_accel_mps2[3];   /* 安装系，含重力 */
extern volatile float    imu_service_gyro_rps[3];     /* 安装系，rad/s */
extern volatile float    imu_service_bias_rps[3];     /* 零偏估计，安装系 */

/* 静止检测：rest_gyr_lp 是 _correct_rest 的观测源（残差 = rest_gyr_lp − bias），
 * 运动刚结束时可能仍带残余；若残差方向错，bias 会被拉偏、yaw 突变。
 * 这两个量在器件坐标系，未做装配归一化。 */
extern volatile uint32_t imu_service_rest;            /* 1=当前判定静止 */
extern volatile float    imu_service_rest_t;          /* 连续静止计时，s */
extern volatile float    imu_service_rest_gyr_lp[3];  /* 低通后的静止参考角速度 */
extern volatile float    imu_service_rest_acc_lp[3];  /* 低通后的静止参考加速度 */

extern volatile uint32_t imu_service_heading_zeroed;  /* 1=yaw 零点已建立 */
extern volatile uint32_t imu_service_heading_quality; /* 0/1/2 */
extern volatile float    imu_service_acc_confidence;  /* 加速度可信度，0..1 */
extern volatile float    imu_service_acc_nis;         /* 最近两维倾斜观测 NIS */
extern volatile uint32_t imu_service_acc_rejected;    /* 1=最近加速度受门控 */
}

#endif /* IMU_MODULE_IMU_IMU_FUSION_H */
