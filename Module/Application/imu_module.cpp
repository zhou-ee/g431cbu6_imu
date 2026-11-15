#include "imu_module.h"
#include "imu_service.h"
#include "pyro_core_def.h"

volatile int32_t imu_module_init_result = 1;

/* 产品装配入口：在调度器启动后的初始化任务中调用。
 * 本函数只负责"把采集链路拉起来"，具体任务与总线装配见 Module/IMU/imu_service.cpp。 */
int imu_module_init(void) {
    if (imu_module_init_result == 0) {
        return 0;
    }

    const pyro::status_t ret = imu_service_start();
    switch (ret) {
        case pyro::PYRO_OK:
            imu_module_init_result = 0;
            break;
        case pyro::PYRO_ALREADY_INIT:
            imu_module_init_result = 0;
            break;
        case pyro::PYRO_NO_MEMORY:
            imu_module_init_result = -2; /* 任务/信号量创建失败（FreeRTOS 堆不足） */
            break;
        case pyro::PYRO_PARAM_ERROR:
            imu_module_init_result = -3; /* 算法任务创建失败 */
            break;
        default:
            imu_module_init_result = -1; /* 器件初始化失败 */
            break;
    }
    return imu_module_init_result;
}
