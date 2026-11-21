#include "pyro_bsp_spi.h"
#include "spi.h"

namespace pyro
{

spi_drv_t &bsp_spi::get_spi1()
{
    /* 函数局部静态：延迟到首次调用才构造，避开 C++ 静态初始化期。 */
    static spi_drv_t instance(&hspi1);
    return instance;
}

} // namespace pyro
