/**
 * @file pyro_bsp_spi.cpp
 * @brief Implementation of the Board Support Package for SPI.
 * SPI 板级支持包实现文件。
 *
 * @version 2.0.0
 * @date 2026-11-19
 */

#include "pyro_bsp_spi.h"
#include "spi.h"

namespace pyro
{

spi_drv_t &bsp_spi::get_spi1()
{
    /* 本类是 spi_drv_t 的唯一构造点（构造函数私有 + friend），因此这里
     * 持有的是 SPI1 这条总线的唯一实例，不需要再回 spi_map() 查重。
     *
     * 函数局部静态的三点保证：
     *   1. C++11 起线程安全的一次性初始化；
     *   2. 首次调用时才构造，必然晚于 main() 里的 MX_SPI1_Init()，
     *      此时 HAL 句柄、DMA 通道、NVIC 均已就绪；
     *   3. 静态存储期，程序生命周期内不析构，DMA 目标地址恒有效。
     *
     * 挂在这条总线上的多个从机（各自一根 CS）共用本引用，
     * 由 spi_drv_t 内部的总线锁串行化。 */
    static spi_drv_t instance(&hspi1);
    return instance;
}

} // namespace pyro
