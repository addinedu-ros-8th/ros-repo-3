cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# 기본 패키지 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)

# JSONCPP (lidar_raw JSON 변환용)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBJSONCPP REQUIRED jsoncpp)

# libgpiod 설정
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# include 디렉토리 설정
include_directories(
  include
  include/mobile_controller
  sdk/include
  sdk/src
  sdk/src/arch/linux
  sdk/src/hal
  sdk/src/net
  ${LIBGPIOD_INCLUDE_DIRS}
  ${LIBJSONCPP_INCLUDE_DIRS}
)

# SLLIDAR SDK 경로 및 소스 파일 설정
set(SLLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
file(GLOB_RECURSE SLLIDAR_SDK_SRC
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/net/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

# ✅ roscar_register_publisher 노드 추가
add_executable(roscar_register_publisher
  ros_nodes/roscar_register_publisher.cpp
)
ament_target_dependencies(roscar_register_publisher
  rclcpp
  shared_interfaces
)

# ✅ roscar_battery_publisher 노드 추가
add_executable(roscar_battery_publisher
  ros_nodes/roscar_battery_publisher.cpp
  sensor_read/battery.cpp
)
ament_target_dependencies(roscar_battery_publisher
  rclcpp
  shared_interfaces
)

# ✅ SensorData 퍼블리셔 (Lidar + IMU + 초음파 통합)
add_executable(sensor_data_publisher
  ros_nodes/sensor_data_publisher.cpp
  sensor_read/imu_sensor.cpp
  sensor_read/ultra_sensor.cpp
  sensor_read/kalman_filter.cpp
  ${SLLIDAR_SDK_SRC}
)
ament_target_dependencies(sensor_data_publisher
  rclcpp
  shared_interfaces
)
target_compile_definitions(sensor_data_publisher PRIVATE SDK_PLATFORM_LINUX)
target_link_libraries(sensor_data_publisher
  ${LIBJSONCPP_LIBRARIES}
  ${LIBGPIOD_LIBRARIES}
)

# 실행 파일 설치
install(TARGETS
  roscar_register_publisher
  roscar_battery_publisher
  sensor_data_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# launch 폴더 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
