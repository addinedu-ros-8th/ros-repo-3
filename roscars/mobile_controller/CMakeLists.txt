cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)

# ✅ include 디렉토리 설정 추가
include_directories(
  include
)

# executable 추가
add_executable(robot_status_publisher ros_nodes/robot_status_publisher.cpp)

ament_target_dependencies(robot_status_publisher
  rclcpp
  std_msgs
  shared_interfaces
)

install(TARGETS
  robot_status_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
