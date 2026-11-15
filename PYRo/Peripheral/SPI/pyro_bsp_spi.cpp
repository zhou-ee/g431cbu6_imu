/**
 * @file pyro_bsp_spi.cpp
 * @brief Implementation of the Board Support Package for SPI.
 * SPI 板级支持包实现文件。
 *
 * @version 1.0.0
 * @date 2026-11-13
 */

#include "pyro_bsp_spi.h"
#include "spi.h"

namespace pyro
{

spi_drv_t &bsp_spi::get_spi1()
{
    /* 复用式单例：句柄表中已有实例（总线实现类静态初始化期创建）则复用，
     * 否则自建并持有。函数局部静态保证线程安全的一次性初始化，
     * 且静态初始化发生在所有全局对象构造之后（main 之前），顺序确定。 */
    static spi_drv_t *cached = pyro::spi_drv_t::get(&hspi1);
    if (nullptr == cached)
    {
        static spi_drv_t owned(&hspi1);
        cached = &owned;
    }
    return *cached;
}

} // namespace pyro
