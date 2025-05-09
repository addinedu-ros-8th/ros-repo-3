cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# 기본 패키지 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)

# libgpiod 설정 추가
find_package(PkgConfig REQUIRED)
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
  ${LIBGPIOD_INCLUDE_DIRS}  # libgpiod include 추가
)

# SLLIDAR SDK 경로 설정
set(SLLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")

# SLLIDAR SDK 소스 파일 수집
file(GLOB_RECURSE SLLIDAR_SDK_SRC
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/net/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

# robot_register_publisher 노드
add_executable(robot_register_publisher ros_nodes/robot_register_publisher.cpp)
ament_target_dependencies(robot_register_publisher
  rclcpp
  shared_interfaces
)

# robot_battery_publisher 노드
add_executable(robot_battery_publisher ros_nodes/robot_battery_publisher.cpp)
ament_target_dependencies(robot_battery_publisher
  rclcpp
  shared_interfaces
)

# robot_lidar_publisher 노드
add_executable(robot_lidar_publisher
  ros_nodes/robot_lidar_publisher.cpp
  ${SLLIDAR_SDK_SRC}
)
ament_target_dependencies(robot_lidar_publisher
  rclcpp
  shared_interfaces
)
target_compile_definitions(robot_lidar_publisher PRIVATE
  SDK_PLATFORM_LINUX
)

# robot_imu_publisher 노드
add_executable(robot_imu_publisher ros_nodes/robot_imu_publisher.cpp)
ament_target_dependencies(robot_imu_publisher
  rclcpp
  shared_interfaces
)

# robot_ultra_publisher 노드 (libgpiod 연동)
add_executable(robot_ultra_publisher ros_nodes/robot_ultra_publisher.cpp)
ament_target_dependencies(robot_ultra_publisher
  rclcpp
  shared_interfaces
)
target_link_libraries(robot_ultra_publisher ${LIBGPIOD_LIBRARIES})  # libgpiod 연결

# mobile_controller 노드
add_executable(mobile_controller ros_nodes/mobile_controller.cpp)
ament_target_dependencies(mobile_controller
  rclcpp
  shared_interfaces
)

# imu_node 노드
add_executable(imu_node ros_nodes/imu_node.cpp
  sensor_read/imu_sensor.cpp
  sensor_read/kalman_filter.cpp)
ament_target_dependencies(imu_node
  rclcpp
  shared_interfaces
)

# 실행 파일 설치
install(TARGETS
  robot_register_publisher
  robot_battery_publisher
  robot_lidar_publisher
  robot_imu_publisher
  robot_ultra_publisher
  mobile_controller
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 폴더 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
