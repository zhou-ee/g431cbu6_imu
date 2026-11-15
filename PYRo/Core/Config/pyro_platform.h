/**
 * @file pyro_platform.h
 * @brief PYRo 移植适配层（平台差异集中管理）。
 *
 * PYRo 上游库以 STM32H723（Cortex-M7，含 ITCM/DTCM）为基准编写，本工程目标为
 * STM32G431（Cortex-M4，单块 32KB SRAM）。两者在内核与外设 HAL 上存在少量差异，
 * 该头文件把这些差异集中成宏，使其余源码与上游保持逐行一致，便于后续同步上游。
 *
 * 集中处理的差异：
 *   1. ISR 代码段属性：H7 把中断服务代码放进 ITCM(.itcm_text)；M4 无 ITCM，直接留在 Flash。
 *   2. FDCAN 接收 FIFO 水位配置：仅 H7 HAL 提供，G4 HAL 无此 API。
 *   3. DMA 堆所在段：H7 需要专用段避开 DTCM；G4 全片 SRAM 均可被 DMA 访问，留在 .bss 即可。
 *
 * @note 该文件依赖芯片宏（STM32H7xx / STM32G4xx），必须在包含芯片 HAL 头
 *       （main.h / stm32xxxx_hal.h）之后包含；同时它是纯宏头文件，可被 C 与 C++ 共用。
 */

#ifndef __PYRO_PLATFORM_H__
#define __PYRO_PLATFORM_H__

/* --------------------------------------------------------------------------
 * 1. 中断服务代码段属性
 * -------------------------------------------------------------------------- */
#if defined(STM32H7xx)
#define PYRO_ISR_TEXT __attribute__((section(".itcm_text")))
#else
/* Cortex-M4（本工程）：无 ITCM，中断服务代码留在 .text 即可 */
#define PYRO_ISR_TEXT
#endif

/* --------------------------------------------------------------------------
 * 2. FDCAN 接收 FIFO 水位设置能力
 *   H7 HAL: HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, N) 可用；
 *   G4 HAL: 无该函数（FIFO 元素数在 HAL_FDCAN_Init 中由 CubeMX 配置决定）。
 * -------------------------------------------------------------------------- */
#if defined(STM32H7xx)
#define PYRO_CAN_HAS_FIFO_WATERMARK 1
#else
#define PYRO_CAN_HAS_FIFO_WATERMARK 0
#endif

/* --------------------------------------------------------------------------
 * 3. DMA 堆所在段
 *   定义 PYRO_DMA_HEAP_SECTION 为段名（如 ".dma_heap"）时把 DMA 堆放到该段；
 *   不定义时留在 .bss —— 本工程 32KB SRAM 全部可被 DMA 访问，无需专用段。
 * -------------------------------------------------------------------------- */
#ifdef PYRO_DMA_HEAP_SECTION
#define PYRO_DMA_HEAP_ATTR __attribute__((section(PYRO_DMA_HEAP_SECTION)))
#else
#define PYRO_DMA_HEAP_ATTR
#endif

#endif // __PYRO_PLATFORM_H__
