# IMU 产品模块（Module → PYRo → CubeMX）
#
# 分层：Application(装配) / IMU(采集 & 融合) / Communication(协议与发布) / Config(产品参数)
# 对外数据接口是 PROTOCOL.md 定义的线上格式。

add_library(IMUProduct OBJECT
        Application/imu_module.cpp
        IMU/imu_capture.cpp
        IMU/imu_fusion.cpp
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
# 该测试只链接 Communication/imu_protocol.cpp 与 PYRo 的 CRC，
# 不引入 HAL/FreeRTOS，因此不进固件目标。
# ------------------------------------------------------------------------------
