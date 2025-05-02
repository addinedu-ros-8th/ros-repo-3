cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# 기본 패키지 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)

# include 디렉토리 설정
include_directories(
  include
  sdk/include
  sdk/src
  sdk/src/arch/linux
  sdk/src/hal
  sdk/src/net
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

# robot_status_publisher 노드
add_executable(robot_status_publisher ros_nodes/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher
  rclcpp
  std_msgs
  shared_interfaces
)

# robot_battery_publisher 노드
add_executable(robot_battery_publisher ros_nodes/robot_battery_publisher.cpp)
ament_target_dependencies(robot_battery_publisher
  rclcpp
  std_msgs
  shared_interfaces
)

# sllidar_node 노드
add_executable(sllidar_node
  ros_nodes/sllidar_node.cpp
  ${SLLIDAR_SDK_SRC}
)

target_compile_definitions(sllidar_node PRIVATE
  SDK_PLATFORM_LINUX
)

ament_target_dependencies(sllidar_node
  rclcpp
  std_msgs
  shared_interfaces
)

# 설치 설정
install(TARGETS
  robot_status_publisher
  robot_battery_publisher
  sllidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
