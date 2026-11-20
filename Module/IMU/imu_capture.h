/**
 * @file    imu_capture.h
 * @brief   采集侧：器件总线绑定 + read() 一拍 + 采样统计。
 *
 * ============================ 职责边界 ============================
 * 本模块只负责"把器件里的原始数据搬到软件环形缓冲"，不碰任何算法状态：
 *   · 持有 SPI 总线对象与 LSM6DSV16x 驱动实例（唯一的硬件入口）
 *   · capture_once()：FIFO 状态 → 等 DMA → 读数据 → TAG 解包 → SPSC publish
 *   · 记录采集侧耗时与错误计数（1 s 窗口）
 *
 * 输出是【器件坐标系】的物理量（单位换算 + 扣静态偏置），
 * 装配方向映射与融合都在 fusion 侧完成——驱动不感知产品如何安装。
 *
 * 由 imu_module.cpp 的 ins_cap 任务每 1 ms 调用 capture_once()。
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
 *  仅可在调度器启动后的任务上下文调用（read 内部要等 DMA 完成信号量）。
 *  @retval PYRO_OK    成功
 *  @retval PYRO_ERROR 器件自检或配置失败 */
pyro::status_t init();

/** 采集一拍：read() + 诊断计数 + 1 s 窗口统计。
 *  内部两段 DMA 等待会挂起任务，不占 CPU。 */
void capture_once();

/** 驱动数据视图（供 fusion 侧读取环形缓冲与解包参数）。
 *  返回引用是安全的：驱动实例生命周期与程序相同，且只有 fusion 侧在读。 */
imu_data_t &data();

/** 归还已消费的样本数（只前移 head，O(1)，不搬数据）。
 *  head 的所有权在消费者侧，但实际存储属于驱动，因此由本模块代理。 */
void release_consumed(uint16_t gyro_count, uint16_t acc_count);

/** 采集侧耗时（1 s 窗口）与轮次。 */
uint32_t cap_rounds();
uint32_t cap_us_max();
uint32_t cap_us_avg();
uint32_t cap_stack_free();

} // namespace capture
} // namespace imu_module

/* ============================ 调试观察量 ============================
 * 定义在本模块的 .cpp。必须放全局作用域（extern "C"），否则 ELF 里的 C 链接
 * 裸名字与调试器按命名空间查找的名字对不上，gdb 会报 Missing ELF symbol。 */
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
