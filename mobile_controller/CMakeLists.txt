cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)

add_executable(robot_status_publisher ros_nodes/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher
  rclcpp
  std_msgs
)

add_executable(robot_status_service ros_nodes/robot_status_service.cpp)
ament_target_dependencies(robot_status_service
  rclcpp
  shared_interfaces
)

install(TARGETS
  robot_status_publisher
  robot_status_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
