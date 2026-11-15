/**
 * @file pyro_bsp_spi.h
 * @brief Board Support Package for SPI instances.
 * SPI 板级支持包头文件。
 *
 * Provides singleton interfaces for specific SPI hardware instances, ensuring
 * safe initialization and isolation from the core driver logic.
 * 提供特定 SPI 硬件实例的单例接口，确保安全的初始化以及与核心驱动逻辑的隔离。
 *
 * @note 移植说明：本工程(STM32G431) CubeMX 只配置了 SPI1（接 LSM6DSV16X），
 *       故仅实现 get_spi1()。
 * @note 与 bsp_uart 的差异：uart_drv_t 需要进程级单例持有常驻接收；SPI 无后台
 *       状态，总线实现类（如 lsm6dsv16x_spi_t）会按注入句柄自建 spi_drv_t 实例。
 *       get_spi1() 因此是"复用式"访问：已注册实例则复用，否则自建并持有，
 *       保证同一 HAL 句柄全局只有一个驱动实例（ISR 路由唯一性）。
 *
 * @version 1.0.0
 * @date 2026-11-13
 */

#ifndef __PYRO_BSP_SPI_H__
#define __PYRO_BSP_SPI_H__

#include "pyro_spi_drv.h"

namespace pyro
{
/**
 * @brief BSP manager class for SPI.
 * SPI 的 BSP 管理类。
 */
class bsp_spi
{
  public:
    /* Instance Getters ------------------------------------------------------*/

    /**
     * @brief Gets the singleton reference for SPI1.
     * 获取 SPI1 的单例引用。
     *
     * @note 复用已注册实例（如 IMU 中间件自建的 spi_drv_t），不存在时自建。
     */
    static spi_drv_t &get_spi1();
};

} // namespace pyro

#endif // __PYRO_BSP_SPI_H__
