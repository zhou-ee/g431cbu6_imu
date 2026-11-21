/**
 * @file pyro_bsp_spi.h
 * @brief SPI 板级支持包：spi_drv_t 的唯一构造入口。
 */

#ifndef __PYRO_BSP_SPI_H__
#define __PYRO_BSP_SPI_H__

#include "pyro_spi_drv.h"

namespace pyro
{
class bsp_spi
{
  public:
    /** 获取 SPI1 的总线驱动引用，该总线上所有从机共用。
     *  首次调用时才构造，必然晚于 main() 中的 MX_SPI1_Init()。 */
    static spi_drv_t &get_spi1();
};

} // namespace pyro

#endif // __PYRO_BSP_SPI_H__
