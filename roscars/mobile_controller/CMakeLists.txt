cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# 기본 패키지 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
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
  ${rclcpp_INCLUDE_DIRS}
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

# ───────────────────────────────────────────────────────────────
# 기존 노드 (예: roscar_register, battery, sensor_data 등)
# ───────────────────────────────────────────────────────────────
add_executable(roscar_register_publisher
  ros_nodes/roscar_register_publisher.cpp
)
ament_target_dependencies(roscar_register_publisher
  rclcpp
  shared_interfaces
)

add_executable(roscar_battery_publisher
  ros_nodes/roscar_battery_publisher.cpp
  sensor_read/battery.cpp
)
ament_target_dependencies(roscar_battery_publisher
  rclcpp
  shared_interfaces
)

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

add_executable(roscar_presure_reader
  ros_nodes/roscar_presure_reader.cpp
)
ament_target_dependencies(roscar_presure_reader
  rclcpp
  std_msgs
)

# ───────────────────────────────────────────────────────────────
# 토픽 구독자 / 발행자 노드 (ros_nodes/)
# ───────────────────────────────────────────────────────────────
# Main → Mobile (구독자)
add_executable(roscar_status_charge_subscriber
  ros_nodes/roscar_status_charge_subscriber.cpp
)
ament_target_dependencies(roscar_status_charge_subscriber
  rclcpp
  shared_interfaces
)

add_executable(roscar_navigation_goal_subscriber
  ros_nodes/roscar_navigation_goal_subscriber.cpp
)
ament_target_dependencies(roscar_navigation_goal_subscriber
  rclcpp
  shared_interfaces
)

add_executable(dashboard_status_update_subscriber
  ros_nodes/dashboard_status_update_subscriber.cpp
)
ament_target_dependencies(dashboard_status_update_subscriber
  rclcpp
  shared_interfaces
)

add_executable(roscar_obstacle_response_subscriber
  ros_nodes/roscar_obstacle_response_subscriber.cpp
)
ament_target_dependencies(roscar_obstacle_response_subscriber
  rclcpp
  shared_interfaces
)

add_executable(roscar_avoidance_cmd_subscriber
  ros_nodes/roscar_avoidance_cmd_subscriber.cpp
)
ament_target_dependencies(roscar_avoidance_cmd_subscriber
  rclcpp
  shared_interfaces
)

add_executable(roscar_precision_stop_cmd_subscriber
  ros_nodes/roscar_precision_stop_cmd_subscriber.cpp
)
ament_target_dependencies(roscar_precision_stop_cmd_subscriber
  rclcpp
  shared_interfaces
)

# Mobile → Main (퍼블리셔)

add_executable(roscar_task_progress_publisher
  ros_nodes/roscar_task_progress_publisher.cpp
)
ament_target_dependencies(roscar_task_progress_publisher
  rclcpp
  shared_interfaces
)

add_executable(roscar_task_complete_publisher
  ros_nodes/roscar_task_complete_publisher.cpp
)
ament_target_dependencies(roscar_task_complete_publisher
  rclcpp
  shared_interfaces
)

add_executable(roscar_precision_stop_result_publisher
  ros_nodes/roscar_precision_stop_result_publisher.cpp
)
ament_target_dependencies(roscar_precision_stop_result_publisher
  rclcpp
  shared_interfaces
)

add_executable(roscar_obstacle_detected_publisher
  ros_nodes/roscar_obstacle_detected_publisher.cpp
)
ament_target_dependencies(roscar_obstacle_detected_publisher
  rclcpp
  shared_interfaces
)


# ───────────────────────────────────────────────────────────────
# 액션 서버 노드 (Mobile Controller → 실제 로직 수행)
# 소스는 action/ 폴더에 위치
# ───────────────────────────────────────────────────────────────

add_executable(start_delivery_server action/start_delivery_server.cpp)
ament_target_dependencies(start_delivery_server
  rclcpp
  rclcpp_action
  shared_interfaces
)

add_executable(move_to_goal_action_server
  action/move_to_goal_action_server.cpp
)
ament_target_dependencies(move_to_goal_action_server
  rclcpp
  rclcpp_action
  shared_interfaces
)

add_executable(scan_inventory_action_server
  action/scan_inventory_action_server.cpp
)
ament_target_dependencies(scan_inventory_action_server
  rclcpp
  rclcpp_action
  shared_interfaces
)

add_executable(maintenance_charge_action_server
  action/maintenance_charge_action_server.cpp
)
ament_target_dependencies(maintenance_charge_action_server
  rclcpp
  rclcpp_action
  shared_interfaces
)

add_executable(security_patrol_action_server
  action/security_patrol_action_server.cpp
)
ament_target_dependencies(security_patrol_action_server
  rclcpp
  rclcpp_action
  shared_interfaces
)

# ───────────────────────────────────────────────────────────────
# 설치 지시
# ───────────────────────────────────────────────────────────────
install(TARGETS
  roscar_register_publisher
  roscar_battery_publisher
  sensor_data_publisher
  roscar_presure_reader

  roscar_status_charge_subscriber
  roscar_navigation_goal_subscriber
  dashboard_status_update_subscriber
  roscar_obstacle_response_subscriber
  roscar_avoidance_cmd_subscriber
  roscar_precision_stop_cmd_subscriber

  roscar_task_progress_publisher
  roscar_task_complete_publisher
  roscar_precision_stop_result_publisher
  roscar_obstacle_detected_publisher

  move_to_goal_action_server
  start_delivery_server
  scan_inventory_action_server
  maintenance_charge_action_server
  security_patrol_action_server

  DESTINATION lib/${PROJECT_NAME}
)

# launch 폴더 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
