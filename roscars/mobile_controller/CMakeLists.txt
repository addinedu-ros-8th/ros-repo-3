cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# 기본 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)

# 노드 추가
add_executable(robot_status_publisher ros_nodes/robot_status_publisher.cpp)

ament_target_dependencies(robot_status_publisher
  rclcpp
  std_msgs
  shared_interfaces  # ✅ 추가: shared_interfaces 의존성 명시
)

# 설치 설정
install(TARGETS
  robot_status_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
