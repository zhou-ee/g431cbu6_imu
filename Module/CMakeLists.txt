# IMU 产品模块（Module → PYRo → CubeMX）
#
# 分层：Application(装配) / IMU(INS 服务与快照) / Communication(协议与发布) / Config(产品参数)
# 对外数据接口是 PROTOCOL.md 定义的线上格式；本目标不依赖任何 HAL 头文件之外的东西。

add_library(IMUProduct OBJECT
        Application/imu_module.cpp
        IMU/ins_service.cpp
        Communication/imu_protocol.cpp
        Communication/imu_publisher.cpp)

target_include_directories(IMUProduct PUBLIC
        Application
        Communication
        Config
        IMU)

# 依赖方向：Module → PYRo（驱动、算法、任务基类、外设封装）→ CubeMX（HAL/FreeRTOS）
target_link_libraries(IMUProduct PUBLIC PYRo)

# ------------------------------------------------------------------------------
# 协议编解码的主机验证（Module/tests/test_protocol.cpp + run_protocol_test.py）
# 该测试只链接 Communication/imu_protocol.cpp，不引入 HAL/FreeRTOS，因此不进固件目标。
# ------------------------------------------------------------------------------
