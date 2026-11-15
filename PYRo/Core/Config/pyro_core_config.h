#ifndef __PYRO_CORE_CONFIG_H__
#define __PYRO_CORE_CONFIG_H__

/* 实例获取宏：使用前需先包含对应的 BSP 头文件（如 pyro_bsp_uart.h）。
 * 上游列出的 UART1/5/7/10 在本 MCU(STM32G431) 上不存在，本工程只开放实际配置的外设。 */

#define PYRO_UART2 pyro::bsp_uart::get_uart2()

#endif // __PYRO_CORE_CONFIG_H__
