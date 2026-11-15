#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 在调度器启动后的产品初始化任务中调用。
 *  0  成功
 * -1  器件初始化失败
 * -2  任务/信号量创建失败（FreeRTOS 堆不足）
 * -3  VQF 配置失败或算法任务创建失败 */
int imu_module_init(void);
/* 供调试器观察启动结果，1 表示尚未完成初始化。 */
extern volatile int32_t imu_module_init_result;

#ifdef __cplusplus
}
#endif

#endif
