cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# PYRo 库（移植子集）
# ==============================================================================
# 源库：E:\Software\Develop\RM\Wheel_leg\PYRo-wl\PYRo  (STM32H723 / Cortex-M7)
# 本工程：STM32G431 / Cortex-M4
#
# 本次仅移植：
#   Peripheral : CAN(FDCAN) / UART(DMA 双缓冲) / DWT(高精度计时) / SPI(阻塞 + DMA 收发)
#   Core       : Lock(mutex/rw_lock) / Task(task_base_t)
#   Core 依赖  : Def(状态码) / ETL(map) / Memory(DMA 堆) —— 上述模块的直接依赖
#   Algorithm  : Matrix / KF / Butterworth / VQF（六轴姿态估计）
#
# 待按产品需要补充：Debug、FSM、Module、ETL 其余部分，
#   Algorithm 其余部分（Kinematics/OLS/PID/...）及 dsppp（本子集算法未使用）。
#   CMSIS-DSP 由根目录 third_party 接入，目标名 CMSISDSP。
# ==============================================================================

add_library(PYRo OBJECT)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

target_sources(PYRo PRIVATE
        # --- Core ---
        Core/ETL/map.cpp
        Core/Lock/pyro_mutex.cpp
        Core/Lock/pyro_rw_lock.cpp
        Core/Task/pyro_task.cpp
        Core/Memory/pyro_core_dma_heap.c
        Core/Memory/pyro_core_mem.cpp

        # --- Peripheral ---
        Peripheral/CAN/pyro_can_drv.cpp
        Peripheral/CAN/pyro_bsp_can.cpp
        Peripheral/DWT/pyro_dwt_drv.cpp
        Peripheral/SPI/pyro_spi_drv.cpp
        Peripheral/SPI/pyro_bsp_spi.cpp
        Peripheral/UART/pyro_uart_drv.cpp
        Peripheral/UART/pyro_bsp_uart.cpp

        # --- Algorithm ---
        # Matrix / VQF：CMSIS-DSP 后端与六轴姿态估计。
        # KF（pyro_algo_kf.h + .tpp）为纯头文件，只需包含路径。
        # 链接自检（可选）：tests/pyro_algo_link_check.cpp
        Algorithm/Matrix/pyro_algo_matrix.cpp
        Algorithm/VQF/pyro_algo_vqf.cpp

        # --- IMU ---
        Component/IMU/imu_base_drv.cpp
        Component/IMU/lsm6dsv16x/lsm6dsv16x_drv.cpp
        Device/lsm6dsv16x_middleware.cpp
)

target_include_directories(PYRo PUBLIC
        # --- Core ---
        Core/Config
        Core/Def
        Core/ETL
        Core/Lock
        Core/Memory
        Core/Task

        # --- Peripheral ---
        Peripheral/CAN
        Peripheral/DWT
        Peripheral/SPI
        Peripheral/UART

        # --- Algorithm ---
        Algorithm/Matrix
        Algorithm/KF
        Algorithm/Butterworth
        Algorithm/VQF

        Component/IMU
        Component/IMU/lsm6dsv16x
        Device
)

# 依赖 CubeMX 生成的接口库（提供 HAL、FreeRTOS、CMSIS 的头文件路径与芯片宏）
# VQF 与器件驱动共用 Component/IMU 内的 imu_data_t 定义。

# CMSISDSP：Matrix 算法的 CMSIS-DSP 后端（arm_mat_* 系列，含 cholesky / 三角求解）
target_link_libraries(PYRo PUBLIC stm32cubemx CMSISDSP)

# ------------------------------------------------------------------------------
# 可选项：把 DMA 堆放进专用段（H7 等需要避开 DTCM 的芯片）
#   G4 的 32KB SRAM 全部可被 DMA 访问，本工程默认不定义 → ucDmaHeap 留在 .bss。
#   如需专用段，取消下一行注释并同步在 .ld 中增加对应的输出段。
# ------------------------------------------------------------------------------
# target_compile_definitions(PYRo PRIVATE PYRO_DMA_HEAP_SECTION=".dma_heap")
