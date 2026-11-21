/**
 * @file    imu_capture.h
 * @brief   采集侧：器件总线绑定 + read() 一拍 + 采样统计。
 *
 * 只负责把器件原始数据搬进软件环形缓冲，不碰算法状态。
 * 输出为器件坐标系物理量；装配映射与融合在 fusion 侧完成。
 */

#ifndef IMU_MODULE_IMU_IMU_CAPTURE_H
#define IMU_MODULE_IMU_IMU_CAPTURE_H

#include "imu_base_drv.h"
#include "pyro_core_def.h"

namespace imu_module
{
namespace capture
{

/** 总线与器件初始化（WHO_AM_I 自检、软复位、量程/ODR、FIFO、时间戳）。
 *  仅可在调度器启动后的任务上下文调用（read 内部要等 DMA 完成信号量）。 */
pyro::status_t init();

/** 采集一拍：read() + 诊断计数 + 1 s 窗口统计。等待期间任务挂起，不占 CPU。 */
void capture_once();

/** 驱动数据视图（供 fusion 读取环形缓冲与解包参数）。 */
imu_data_t &data();

/** 采集数据中断代次，与两路环形索引在同一调度器挂起窗口内读取。 */
uint32_t discontinuities();

/** 归还已消费的样本数（只前移 head，O(1)）。 */
void release_consumed(uint16_t gyro_count, uint16_t acc_count);

/** 采集侧耗时（1 s 窗口）与轮次。 */
uint32_t cap_rounds();
uint32_t cap_us_max();
uint32_t cap_us_avg();
uint32_t cap_stack_free();

} // namespace capture
} // namespace imu_module

/* 调试观察量：必须放全局作用域（extern "C"），否则调试器按命名空间查找
 * 时对不上 ELF 里的裸名字。定义在本模块的 .cpp。 */
extern "C" {
extern volatile uint32_t imu_service_read_ok;        /* read() 返回 true 的次数 */
extern volatile uint32_t imu_service_cap_rounds;     /* 采集轮次（1 kHz 硬周期） */
extern volatile uint32_t imu_service_cap_us_max;     /* 单拍 read() 峰值耗时，µs */
extern volatile uint32_t imu_service_cap_us_avg;     /* 单拍 read() 平均耗时，µs */
extern volatile uint32_t imu_service_cap_stack_free; /* 采集任务栈余量（词） */
extern volatile int32_t  imu_service_init_result;    /* 采集初始化结果 0=成功 */
extern volatile uint32_t imu_service_heap_free_min;  /* 任务拉起后的 FreeRTOS 堆余量 */
}

#endif /* IMU_MODULE_IMU_IMU_CAPTURE_H */
