/**
 * @file pyro_bsp_spi.h
 * @brief SPI 板级支持包。
 *
 * bsp_spi 是 spi_drv_t 的唯一构造入口（构造函数私有 + friend）。
 * 挂在同一总线上的多颗从机共用同一个引用，详见同目录 README.md。
 *
 * @version 2.0.0
 * @date 2026-11-20
 */

#ifndef __PYRO_BSP_SPI_H__
#define __PYRO_BSP_SPI_H__

#include "pyro_spi_drv.h"

namespace pyro
{
/**
 * @brief SPI 的 BSP 管理类，同时是 spi_drv_t 的唯一工厂。
 */
class bsp_spi
{
  public:
    /**
     * @brief 获取 SPI1 的总线驱动引用（该总线上所有从机共用）。
     * @note 首次调用时才构造，必然晚于 main() 中的 MX_SPI1_Init()。
     */
    static spi_drv_t &get_spi1();
};

} // namespace pyro

#endif // __PYRO_BSP_SPI_H__
