/**
* @file pyro_bsp_uart.h
 * @brief Board Support Package for UART instances.
 * 串口板级支持包头文件。
 *
 * Provides singleton interfaces for specific UART hardware instances, ensuring
 * safe initialization and isolation from the core driver logic.
 * 提供特定串口硬件实例的单例接口，确保安全的初始化以及与核心驱动逻辑的隔离。
 *
 * @note 移植说明：上游(H723)提供 uart1/uart5/uart7/uart10；本工程(STM32G431)
 *       CubeMX 只配置了 USART2，故仅实现 get_uart2()。
 *
 * @author Lucky
 * @version 1.0.1
 * @date 2026-03-25
 */

#ifndef __PYRO_BSP_UART_H__
#define __PYRO_BSP_UART_H__

#include "pyro_uart_drv.h"

namespace pyro
{
/**
 * @brief BSP manager class for UART.
 * 串口的 BSP 管理类。
 */
class bsp_uart
{
public:
    /* Instance Getters ------------------------------------------------------*/

    /**
     * @brief Gets the singleton reference for UART2.
     * 获取 UART2 的单例引用。
     */
    static uart_drv_t& get_uart2();

    /* Initialization --------------------------------------------------------*/

    /**
     * @brief Initializes all hardware UART instances.
     * 初始化所有硬件串口实例。
     *
     * @note 前置条件：CubeMX 中该串口需要分配 RX DMA 通道
     *       （USART2_RX → DMAx_ChannelN，并打开对应 DMA 中断），
     *       否则 enable_rx_dma() 会因 hdmarx 为空而返回 PYRO_ERROR。
     */
    static void init_all();
};

} // namespace pyro

#endif // __PYRO_BSP_UART_H__
