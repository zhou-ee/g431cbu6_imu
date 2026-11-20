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

/* ============================ 固件版本 ============================
 * 语义化版本：主.次.修订
 *   主 —— 对外协议或行为不兼容变更
 *   次 —— 向后兼容的功能新增（本版：ESKF 拆分为独立 KF 组件）
 *   修订 —— 向后兼容的问题修正或性能优化
 *
 * 修订历史：
 *   1.0.0  首个正式版：双任务 + SPSC + 信号量驱动发布
 *   1.0.1  VQF 静止态耗时优化（Joseph 块化 / 检查精简 / trig 复用）
 *   1.1.0  ESKF 抽为 PYRo/Algorithm/KF 下的独立组件，与 ekf/lkf 并列
 *   1.2.0  发布层与器件解耦：编码系数改由协议层按模块量程自行定义，
 *          角速度发送算法扣零偏后的最终值，移除 diag 与丢样代次校验
 *
 * 版本号不参与线上协议（PROTOCOL.md 明确本版消息不含版本字段），
 * 仅用于固件标识与问题追溯。 */
#define IMU_MODULE_VERSION_MAJOR 1
#define IMU_MODULE_VERSION_MINOR 2
#define IMU_MODULE_VERSION_PATCH 0
#define IMU_MODULE_VERSION_STRING "1.2.0"

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
