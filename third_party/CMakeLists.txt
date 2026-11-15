set(FASTBUILD ON CACHE BOOL "CMSIS-DSP aggregate builds for Cortex-M")
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16 kernels on Cortex-M4F")

add_subdirectory(CMSIS-DSP)
target_include_directories(CMSISDSP PRIVATE
    "${PROJECT_SOURCE_DIR}/CubeMX/Drivers/CMSIS/Include")

# CMSIS-DSP 的矩阵/滤波内核必须开优化，否则 Debug 构建下 arm_mat_* 是 -O0，
# 单次 3x3 求逆可慢到数百微秒，直接把 VQF 实时性打穿（实测 175 us/样本）。
# 这些是库代码、无需调试，单独提优化级别不影响应用侧的调试体验。
# 同时关闭本目标里的浮点格式告警噪声：库源码在严格告警下会有大量既有告警。
target_compile_options(CMSISDSP PRIVATE -O2 -ffast-math)

# 覆盖 Debug 追加的 -O0/-g3：CMake 的 Debug 预设会把 -g3 追加在 -O2 之后，
# 这里显式补一次 -O2 让后者生效（CMake 选项按出现顺序后者优先）。
target_compile_options(CMSISDSP PRIVATE $<$<CONFIG:Debug>:-O2>)
