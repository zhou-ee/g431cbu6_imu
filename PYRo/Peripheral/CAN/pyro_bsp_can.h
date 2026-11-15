/**
 * @file pyro_bsp_can.h
 * @brief Board Support Package for CAN instances.
 * CAN板级支持包头文件。
 *
 * Provides static interfaces for specific CAN hardware instances, ensuring
 * safe initialization and isolation from the core driver logic.
 * 提供特定CAN硬件实例的静态接口，确保安全的初始化以及与核心驱动逻辑的隔离。
 *
 * @note 移植说明：上游(H723)提供 can1/can2/can3；本工程(STM32G431) CubeMX 只配置了
 *       FDCAN1，故仅实现 get_can1()，can2/can3 的查询返回 nullptr。
 *
 * @author Antigravity
 * @version 1.0.1
 * @date 2026-07-03
 */

#ifndef __PYRO_BSP_CAN_H__
#define __PYRO_BSP_CAN_H__

#include "pyro_can_drv.h"

namespace pyro
{

class bsp_can
{
public:
    enum which_can
    {
        can1,
        can2,
        can3
    };

    static can_drv_t& get_can1();

    static can_drv_t* get_can(which_can which);

    static status_t init_all();
};

} // namespace pyro

#endif // __PYRO_BSP_CAN_H__
