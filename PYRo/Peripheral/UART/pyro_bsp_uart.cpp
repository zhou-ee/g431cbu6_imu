/**
* @file pyro_bsp_uart.cpp
 * @brief Implementation of the Board Support Package for UART.
 * 串口板级支持包实现文件。
 *
 * @author Lucky
 * @version 1.0.1
 * @date 2026-03-25
 */

#include "pyro_bsp_uart.h"
#include "usart.h"

namespace pyro
{

// Using local static variables to ensure a safe initialization order.
// 使用局部静态变量以确保安全的初始化顺序。

uart_drv_t& bsp_uart::get_uart2()
{
    /* 接收缓冲区大小：每个串口占用 2 × 512 字节 DMA 堆（见 Core/Memory） */
    static uart_drv_t instance(&huart2, 512);
    return instance;
}

void bsp_uart::init_all()
{
    // Calling get_uartX() triggers the first-time construction of the instance.
    // 调用 get_uartX() 将触发实例的首次构造。
    get_uart2().enable_rx_dma();
}

} // namespace pyro
