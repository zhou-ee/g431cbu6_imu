/**
 * @file    imu_service.h
 * @brief   IMU 采集 + VQF 姿态融合服务（任务层）。
 *
 * 单个常驻任务按 1 ms 固定周期顺序完成：读取一拍 → VQF 融合 → 归还样本 →
 * 发布姿态快照。任务内部不对外暴露"状态查询/FIFO/直读"这些细节，
 * 上层只通过 imu_service_attitude() 取姿态、或 read_imu_data() 取原始量。
 *
 * 为什么是单任务：双任务（采集 + 算法）会导致环形缓冲 head 在处理期间漂移，
 * 读到错位样本、时间戳倒退，VQF 报 PARAM_ERROR；用临界区保护又会屏蔽
 * SPI/DMA 中断，把采集周期从 1000 Hz 拖到 254 Hz。详细论证见 .cpp 文件头。
 */

#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "imu_base_drv.h"
#include "pyro_algo_vqf.h"
#include "pyro_core_def.h"

/* 姿态结果快照。四元数顺序 [w,x,y,z]，表示传感器安装坐标系到 Z-up 参考系。
 * valid=false 表示 VQF 尚未产生有效姿态，此时四元数为单位四元数。 */
struct imu_attitude_t
{
    float quat[4];     /* [w,x,y,z] */
    float bias[3];     /* 陀螺零偏估计，rad/s */
    float roll;        /* ° */
    float pitch;       /* ° */
    float yaw;         /* ° */
    float rest_t;      /* 连续静止计时，s */
    bool  rest;        /* 当前判定为静止 */
    bool  valid;       /* VQF 已配置且至少完成一次更新 */
};

/* 启动采集链路：初始化器件与 VQF，创建常驻任务。
 * 仅可在调度器启动后的任务上下文调用。
 * @retval PYRO_OK           成功
 * @retval PYRO_ERROR        器件初始化失败
 * @retval PYRO_PARAM_ERROR  VQF 配置失败
 * @retval PYRO_NO_MEMORY    任务创建失败（FreeRTOS 堆不足）
 * @retval PYRO_ALREADY_INIT 重复调用 */
pyro::status_t imu_service_start();

/* 解包后的原始数据（只读视图）。 */
imu_data_t &imu_service_data();

/* 姿态结果快照（按值返回，且保证是某一次完整更新的结果）。
 * 可在任意任务上下文调用；若写入方正在更新，内部会重试，不会返回撕裂数据。 */
imu_attitude_t imu_service_attitude();

/* ---------------- 调试观测（调试器 watch 用，零依赖，可随时删除） ---------------- */
extern "C" {
extern volatile int32_t  imu_service_init_result;   /* 0=器件就绪 1=初始化失败 -1=未运行 */
extern volatile int32_t  imu_service_vqf_ready;     /* 1=VQF 配置成功 */
extern volatile uint32_t imu_service_read_ok;       /* read() 返回 true 的累计次数 */
extern volatile uint32_t imu_service_algo_rounds;   /* 实际执行融合的轮数 */
extern volatile uint32_t imu_service_vqf_consumed;  /* VQF 累计消费的样本数（两路合计）*/
extern volatile uint32_t imu_service_err_bus;       /* 总线失败次数 */
extern volatile uint32_t imu_service_err_data;      /* FIFO 溢出 / 未知 TAG / 缓冲覆盖 */
extern volatile uint32_t imu_service_err_timeout;   /* 等完成通知超时次数 */
extern volatile uint32_t imu_service_vqf_err;       /* VQF 返回 ERROR 的次数 */
extern volatile uint32_t imu_service_vqf_warn;      /* VQF 返回 WARNING 的次数 */
extern volatile int32_t  imu_service_vqf_status;    /* 最近一次 VQF update 的返回值 */
extern volatile uint32_t imu_service_vqf_us_max;    /* 单批 VQF 耗时峰值（µs，每秒刷新）*/
extern volatile uint32_t imu_service_drop_samples;  /* 保留字段（单任务下恒 0）*/
extern volatile uint32_t imu_service_heap_free_min; /* 任务拉起后的 FreeRTOS 堆余量（字节）*/
extern volatile float    imu_service_accel_mps2[3]; /* 最新加速度（m/s²）*/
extern volatile float    imu_service_gyro_rps[3];   /* 最新角速度（rad/s）*/
extern volatile float    imu_service_quat[4];       /* [w,x,y,z] */
extern volatile float    imu_service_quat_norm;     /* 应恒 ≈ 1.0 */
extern volatile float    imu_service_roll;          /* ° */
extern volatile float    imu_service_pitch;         /* ° */
extern volatile float    imu_service_yaw;           /* ° */
extern volatile float    imu_service_bias_rps[3];   /* 零偏估计，rad/s */
extern volatile uint32_t imu_service_rest;          /* 1=当前静止 */
extern volatile float    imu_service_rest_t;        /* 静止计时，s */
}

#endif
