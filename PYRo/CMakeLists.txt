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
        # CRC：UART 帧使用其中的 crc16_ccitt_false（产品协议）。
        # 链接自检（可选）：tests/pyro_algo_link_check.cpp
        Algorithm/Matrix/pyro_algo_matrix.cpp
        Algorithm/VQF/pyro_algo_vqf_eskf.cpp
        Algorithm/CRC/pyro_crc.cpp

        # --- IMU ---
        Component/IMU/imu_base_drv.cpp
        Component/IMU/lsm6dsv16x/lsm6dsv16x_drv.cpp
        Device/lsm6dsv16x_middleware.cpp
        Component/IMU/BMI088/BMI088_driver.cpp
        Device/BMI088/BMI088_middleware.cpp
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
        Algorithm/CRC

        Component/IMU
        Component/IMU/lsm6dsv16x
        Component/IMU/BMI088
        Device
        Device/BMI088
)

# 依赖 CubeMX 生成的接口库（提供 HAL、FreeRTOS、CMSIS 的头文件路径与芯片宏）
# VQF 与器件驱动共用 Component/IMU 内的 imu_data_t 定义。

# CMSISDSP：Matrix 算法的 CMSIS-DSP 后端（arm_mat_* 系列，含 cholesky / 三角求解）
target_link_libraries(PYRo PUBLIC stm32cubemx CMSISDSP)

# ------------------------------------------------------------------------------
# Algorithm 目录下的源码单独用 -O2 编译（其余目标保持外层配置，Debug 即 -O0）
#
# 原因：VQF 的零偏卡尔曼滤波每样本要跑 3x3 矩阵求逆、协方差更新，以及多个
# Butterworth 二阶低通的变步长递推（含 double 版 exp/cos/sin）。这些代码在
# -O0 下实测单样本高达约 110~230 us，而 1920 Hz 双路的实时预算只有
# 520 us/样本（7680 Hz 时更是 65 us），直接打穿实时性。
#
# 只对 Algorithm 生效的好处：
#   - 工程其余部分（驱动、协议、业务层）仍是 -O0，可正常单步调试；
#   - 算法是纯数值计算，不需要单步跟踪，优化不影响调试体验；
#   - 不必为了一个模块把整个 PYRo 提到 -O2。
#
# 注意：KF（pyro_algo_kf.h + .tpp）与 Butterworth 是纯头文件，其代码由
# pyro_algo_vqf_eskf.cpp 实例化，所以给该文件加 -O2 即可覆盖到它们。
# ------------------------------------------------------------------------------
set(PYRO_ALGO_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/Matrix/pyro_algo_matrix.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/VQF/pyro_algo_vqf_eskf.cpp)

set_source_files_properties(${PYRO_ALGO_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-O2")

# ------------------------------------------------------------------------------
# 可选项：把 DMA 堆放进专用段（H7 等需要避开 DTCM 的芯片）
#   G4 的 32KB SRAM 全部可被 DMA 访问，本工程默认不定义 → ucDmaHeap 留在 .bss。
#   如需专用段，取消下一行注释并同步在 .ld 中增加对应的输出段。
# ------------------------------------------------------------------------------
# target_compile_definitions(PYRo PRIVATE PYRO_DMA_HEAP_SECTION=".dma_heap")
