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

/** 采集一拍：read() + 首帧物理量缓存 + 诊断计数 + 1 s 窗口统计。
 *  内部两段 DMA 等待会挂起任务，不占 CPU。 */
void capture_once();

/** 驱动数据视图（供 fusion 侧读取环形缓冲与解包参数）。
 *  返回引用是安全的：驱动实例生命周期与程序相同，且只有 fusion 侧在读。 */
imu_data_t &data();

/** 归还已消费的样本数（只前移 head，O(1)，不搬数据）。
 *  head 的所有权在消费者侧，但实际存储属于驱动，因此由本模块代理。 */
void release_consumed(uint16_t gyro_count, uint16_t acc_count);

/** 加速度 / 角速度的灵敏度（m/s²/LSB、rad/s/LSB），封包反向换算要用。
 *  与驱动正向解包同源，换量程时不必在别处再维护一份。 */
float acc_mps2_per_lsb();
float gyro_rps_per_lsb();

/** 驱动解包时扣除的静态零偏（器件系，rad/s）。封包时要加回去。 */
const imu_cali_t &cali();

/** 最新一帧解包物理量（器件坐标系）。允许陈旧一拍——
 *  协议取"最新驱动物理量"语义，与本拍融合结果同组封包。 */
const float *last_acc_dev();
const float *last_gyro_dev();

/** 采集侧诊断（累计）与耗时（1 s 窗口）。 */
struct diag_t
{
    uint32_t err_bus;
    uint32_t err_data;
    uint32_t err_timeout;
    uint32_t err_ovr;
    uint32_t err_backpressure;
};
diag_t diag();
uint32_t cap_rounds();
uint32_t cap_us_max();
uint32_t cap_us_avg();
uint32_t cap_stack_free();

/** 清零累计错误计数（清的是驱动内部计数器，不是镜像量）。
 *
 * 用途：上电启动期采集任务先于融合任务跑起来，缓冲会被瞬间填满，
 * 于是每次上电都会固定产生几十次 err_backpressure。这段属于启动瞬态、
 * 不代表稳态能力，会让"持续堆积"这个判据失效。
 * 因此在 fusion::init()（消费侧就绪）之后调用一次，使该计数只反映稳态。
 *
 * @note 只清错误计数，不动 cap_rounds / read_ok 等进度量。 */
void clear_error_counters();

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
extern volatile uint32_t imu_service_err_bus;        /* 总线失败次数 */
extern volatile uint32_t imu_service_err_timeout;    /* 等完成通知超时次数 */
extern volatile uint32_t imu_service_err_data;       /* 数据异常合计 */
extern volatile uint32_t imu_service_err_ovr;        /* 其中：器件 FIFO 溢出 */
extern volatile uint32_t imu_service_err_backpressure; /* 其中：缓冲满丢新样 */
extern volatile int32_t  imu_service_init_result;    /* 采集初始化结果 0=成功 */
extern volatile uint32_t imu_service_heap_free_min;  /* 任务拉起后的 FreeRTOS 堆余量 */
}

#endif /* IMU_MODULE_IMU_IMU_CAPTURE_H */
