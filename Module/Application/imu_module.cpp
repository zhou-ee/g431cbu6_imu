/**
 * @file    imu_module.cpp
 * @brief   产品装配入口：按固定顺序拉起 INS 服务与发布任务。
 *
 * 顺序不可交换：
 *   ① INS 服务先起来——它完成器件初始化与算法配置，之后 snapshot() 才有意义；
 *   ② 发布任务后起来——它只依赖快照，发布失败不应阻止采集/融合（板级调试时
 *      仍能通过调试器读到姿态）。
 */

#include "imu_module.h"

#include "imu_publisher.h"
#include "ins_service.h"
#include "pyro_core_def.h"

volatile int32_t imu_module_init_result = 1;

extern "C" int imu_module_init(void)
{
    if (imu_module_init_result == 0)
    {
        return 0;
    }

    /* ① 采集 + 融合：器件初始化、算法配置、常驻任务 */
    const pyro::status_t ins = imu_module::ins_service_t::instance().init();
    switch (ins)
    {
        case pyro::PYRO_OK:
        case pyro::PYRO_ALREADY_INIT:
            break;
        case pyro::PYRO_NO_MEMORY:
            imu_module_init_result = -2;
            return imu_module_init_result;
        case pyro::PYRO_PARAM_ERROR:
            imu_module_init_result = -3;
            return imu_module_init_result;
        default:
            imu_module_init_result = -1;
            return imu_module_init_result;
    }

    /* ② 发布：CAN/UART 传输层与 1 kHz 发布任务 */
    const pyro::status_t pub = imu_module::publisher_t::instance().init();
    if (pub != pyro::PYRO_OK && pub != pyro::PYRO_ALREADY_INIT)
    {
        imu_module_init_result = -4;
        return imu_module_init_result;
    }

    imu_module_init_result = 0;
    return imu_module_init_result;
}
