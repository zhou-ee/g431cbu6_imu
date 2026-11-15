/**
 * @file pyro_bsp_can.cpp
 * @brief Implementation of the Board Support Package for CAN.
 * CAN板级支持包实现文件。
 *
 * @author Antigravity
 * @version 1.0.1
 * @date 2026-07-03
 */

#include "pyro_bsp_can.h"
#include "fdcan.h"

namespace pyro
{

can_drv_t& bsp_can::get_can1()
{
    static can_drv_t instance(&hfdcan1);
    return instance;
}

can_drv_t* bsp_can::get_can(const which_can which)
{
    switch (which)
    {
        case can1:
            return &get_can1();
        /* 本工程未配置 FDCAN2/FDCAN3（G431 只有 1 个 FDCAN）。
         * 若后续 CubeMX 中启用，请在此补充 get_can2()/get_can3() 并返回其实例。 */
        case can2:
        case can3:
        default:
            return nullptr;
    }
}

status_t bsp_can::init_all()
{
    CHECK_PYRO_RET(get_can1().init());
    CHECK_PYRO_RET(get_can1().start());

    return PYRO_OK;
}

} // namespace pyro
