add_library(IMUProduct OBJECT
        Application/imu_module.cpp
        IMU/imu_service.cpp)

target_include_directories(IMUProduct PUBLIC
        Application
        IMU)

target_link_libraries(IMUProduct PUBLIC PYRo)
