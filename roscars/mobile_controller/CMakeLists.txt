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

# executable 추가 - robot_status_publisher
add_executable(robot_status_publisher ros_nodes/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher
  rclcpp
  std_msgs
  shared_interfaces
)

# executable 추가 - robot_battery_publisher
add_executable(robot_battery_publisher ros_nodes/robot_battery_publisher.cpp)
ament_target_dependencies(robot_battery_publisher
  rclcpp
  std_msgs
  shared_interfaces
)

# executable 추가 - lidar
add_executable(lidar_node ros_nodes/lidar.cpp)
ament_target_dependencies(lidar_node
  rclcpp
  std_msgs
  shared_interfaces
)

# 설치 경로 설정
install(TARGETS
  robot_status_publisher
  robot_battery_publisher
  lidar_node  # ✅ lidar_node 추가됨
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
