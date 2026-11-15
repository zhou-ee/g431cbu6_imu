/**
 * @file    imu_service.h
 * @brief   IMU 采集与解算服务（任务层）。
 *
 * 分两个任务，职责单一：
 *   capture：1 ms 一拍调用 imu_base_drv::read()。该接口内部自行完成
 *            "状态查询 → 等完成中断唤醒 → 数据读取 → 等完成中断唤醒 → 解包"，
 *            本任务不感知数据来自 FIFO 还是直读，也不感知状态字节。
 *   algo   ：由 capture 通过二值信号量唤醒，取走本拍解出的样本并归还环形缓冲。
 *            姿态融合留待下一阶段接入，当前只做"唤醒 → 取数 → 归还"打通链路。
 *
 * 唤醒关系（单向，capture → algo）：
 *   SPI DMA 完成中断 ──唤醒──► capture 任务 ──give──► algo 任务
 */

#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "imu_base_drv.h"
#include "pyro_core_def.h"

/* 启动采集链路：创建同步量、初始化器件、拉起两个常驻任务。
 * 仅可在调度器启动后的任务上下文调用（内部使用信号量 / HAL_Delay / DMA 等待）。
 * @retval PYRO_OK          成功
 * @retval PYRO_ERROR       器件初始化失败
 * @retval PYRO_NO_MEMORY   采集任务创建失败
 * @retval PYRO_ALREADY_INIT 重复调用
 * @retval PYRO_PARAM_ERROR 算法任务创建失败 */
pyro::status_t imu_service_start();

/* 解包结果（只读视图）。算法任务取样本用，未来 VQF 也接这里。 */
imu_data_t &imu_service_data();

/* ---------------- 调试观测（调试器 watch 用，零依赖，可随时删除） ---------------- */
extern "C" {
extern volatile int32_t  imu_service_init_result;   /* 0=器件就绪 1=初始化失败 -1=未运行 */
extern volatile uint32_t imu_service_read_ok;       /* read() 返回 true 的累计次数 */
extern volatile uint32_t imu_service_algo_rounds;   /* 算法任务被唤醒的累计次数 */
extern volatile uint32_t imu_service_algo_samples;  /* 算法任务累计归还的样本数（两路合计）*/
extern volatile uint32_t imu_service_err_bus;       /* 总线失败次数 */
extern volatile uint32_t imu_service_err_data;      /* FIFO 溢出 / 未知 TAG / 缓冲覆盖 */
extern volatile uint32_t imu_service_err_timeout;   /* 等完成通知超时次数 */
extern volatile float    imu_service_accel_mps2[3]; /* 最新加速度（m/s²）*/
extern volatile float    imu_service_gyro_rps[3];   /* 最新角速度（rad/s）*/
extern volatile uint32_t imu_service_algo_ts;       /* 算法任务看到的最新器件时间戳 */
}

#endif
