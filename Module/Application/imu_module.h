/**
 * @file    imu_module.h
 * @brief   IMU 产品模块的对外入口（C 接口，供 CubeMX 生成的启动代码调用）。
 *
 * 分层与职责（依赖方向 Module → PYRo → CubeMX）：
 *   Application/  本文件：初始化顺序与产品运行状态
 *   IMU/          imu_capture（器件+采集）· imu_fusion（融合+归零+快照）
 *   Communication/imu_protocol（纯编解码）+ imu_publisher（1 kHz CAN/UART 发布）
 *   Config/       编译期产品参数与装配方向映射
 *
 * 对外数据接口是 Module/PROTOCOL.md 定义的线上格式，不是本文件的函数。
 */

#ifndef IMU_MODULE_APPLICATION_IMU_MODULE_H
#define IMU_MODULE_APPLICATION_IMU_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 在调度器启动后的产品初始化任务中调用（内部会创建常驻任务并等待 DMA 信号量）。
 *  @retval  0 成功（采集/融合与发布均已就绪）
 *  @retval -1 器件或总线初始化失败
 *  @retval -2 任务创建失败（FreeRTOS 堆不足）
 *  @retval -3 算法配置失败
 *  @retval -4 发布层初始化失败（CAN/UART）
 *  失败原因同时体现在各层的调试观测量里。 */
int imu_module_init(void);

/** 供调试器观察启动结果；1 表示尚未完成初始化。 */
extern volatile int32_t imu_module_init_result;

#ifdef __cplusplus
}
#endif

#endif /* IMU_MODULE_APPLICATION_IMU_MODULE_H */
