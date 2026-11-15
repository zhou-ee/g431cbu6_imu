set(FASTBUILD ON CACHE BOOL "CMSIS-DSP aggregate builds for Cortex-M")
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16 kernels on Cortex-M4F")

add_subdirectory(CMSIS-DSP)
target_include_directories(CMSISDSP PRIVATE
    "${PROJECT_SOURCE_DIR}/CubeMX/Drivers/CMSIS/Include")
