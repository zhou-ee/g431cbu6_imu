include("${CMAKE_CURRENT_LIST_DIR}/../CubeMX/cmake/gcc-arm-none-eabi.cmake")

# 根目录构建使用 CubeMX 内的 G431 链接脚本。
get_filename_component(IMU_MODULE_LINKER_SCRIPT
    "${CMAKE_CURRENT_LIST_DIR}/../CubeMX/STM32G431xx_FLASH.ld" ABSOLUTE)
set(CMAKE_EXE_LINKER_FLAGS
    "${TARGET_FLAGS} -T \"${IMU_MODULE_LINKER_SCRIPT}\" --specs=nano.specs -Wl,-Map=IMU_Module.map -Wl,--gc-sections -Wl,--print-memory-usage")
