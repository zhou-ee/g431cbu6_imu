/**
 * @file pyro_bsp_spi.h
 * @brief Board Support Package for SPI instances.
 * SPI 板级支持包头文件。
 *
 * 【唯一入口】bsp_spi 是 spi_drv_t 的唯一构造入口（构造函数私有 + friend）。
 *   一个 HAL 外设句柄对应一条总线，全局只有一个 spi_drv_t 实例；
 *   挂在该总线上的多个从机（各自一根 CS）共用同一个引用：
 *
 *     pyro::spi_drv_t &bus = pyro::bsp_spi::get_spi1();
 *     lsm6dsv16x_spi_t imu(bus, GPIOA, GPIO_PIN_4);    // 从机 A
 *     lsm6dsv16x_spi_t aux(bus, GPIOA, GPIO_PIN_15);   // 从机 B（同一总线）
 *
 * @note 移植说明：本工程(STM32G431) CubeMX 只配置了 SPI1（接 LSM6DSV16X），
 *       故仅实现 get_spi1()。新增 SPI 外设时在此加 get_spiX()。
 *
 * @version 2.0.0
 * @date 2026-11-19
 */

#ifndef __PYRO_BSP_SPI_H__
#define __PYRO_BSP_SPI_H__

#include "pyro_spi_drv.h"

namespace pyro
{
/**
 * @brief BSP manager class for SPI.
 * SPI 的 BSP 管理类（同时是 spi_drv_t 的唯一工厂）。
 */
class bsp_spi
{
  public:
    /* Instance Getters ------------------------------------------------------*/

    /**
     * @brief Gets the singleton reference for SPI1.
     * 获取 SPI1 的总线驱动引用（该总线上所有从机共用）。
     *
     * @note 首次调用时才构造，必然晚于 main() 中的 MX_SPI1_Init()。
     * @retval 该总线的唯一 spi_drv_t 引用（程序生命周期内恒有效）。
     */
    static spi_drv_t &get_spi1();
};

} // namespace pyro

#endif // __PYRO_BSP_SPI_H__
