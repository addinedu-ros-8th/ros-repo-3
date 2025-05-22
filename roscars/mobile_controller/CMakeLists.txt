cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# ───────────────────────────────────────────────────────────────
# 1. 패키지 설정
# ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# JSONCPP (LiDAR 데이터 JSON 변환용)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBJSONCPP REQUIRED jsoncpp)

# libgpiod 설정 (초음파 센서 인터페이스)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# ───────────────────────────────────────────────────────────────
# 2. include 디렉토리 설정
# ───────────────────────────────────────────────────────────────
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

<<<<<<< Updated upstream
# SLLIDAR SDK 소스 파일 설정
=======
# ───────────────────────────────────────────────────────────────
# 3. SLLIDAR SDK 소스 파일 모음
# ───────────────────────────────────────────────────────────────
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
# 유틸리티 노드
# ───────────────────────────────────────────────────────────────

# TEB Local Planner
add_executable(teb_local_planner drive/teb_local_planner.cpp)
ament_target_dependencies(teb_local_planner
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  shared_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(teb_local_planner
  ${LIBJSONCPP_LIBRARIES}
  ${LIBGPIOD_LIBRARIES}
)

# Roscar Register Publisher
add_executable(roscar_register_publisher ros_nodes/roscar_register_publisher.cpp)
ament_target_dependencies(roscar_register_publisher
  rclcpp
  shared_interfaces
=======
# 4. 액션 서버 노드들
# ───────────────────────────────────────────────────────────────
file(GLOB ACTION_SRC
  action/*.cpp
>>>>>>> Stashed changes
)
foreach(src ${ACTION_SRC})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  ament_target_dependencies(${name}
    rclcpp
    rclcpp_action
    std_msgs
    shared_interfaces
  )
endforeach()

<<<<<<< Updated upstream
# Battery Publisher
add_executable(roscar_battery_publisher
  ros_nodes/roscar_battery_publisher.cpp
  sensor_read/battery.cpp
)
ament_target_dependencies(roscar_battery_publisher
  rclcpp
  shared_interfaces
)

target_link_libraries(roscar_battery_publisher
  ${LIBGPIOD_LIBRARIES}
)

# Sensor Data Publisher
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

# Pressure Reader
add_executable(roscar_presure_reader ros_nodes/roscar_presure_reader.cpp)
ament_target_dependencies(roscar_presure_reader
  rclcpp
  std_msgs
)

# LiDAR Publisher
add_executable(roscar_lidar_publisher
  ros_nodes/roscar_lidar_publisher.cpp
  ${SLLIDAR_SDK_SRC}
)
ament_target_dependencies(roscar_lidar_publisher
  rclcpp
  shared_interfaces
)
target_compile_definitions(roscar_lidar_publisher PRIVATE SDK_PLATFORM_LINUX)

target_link_libraries(roscar_lidar_publisher
  ${LIBJSONCPP_LIBRARIES}
)

# IMU Publisher
add_executable(roscar_imu_publisher
  ros_nodes/roscar_imu_publisher.cpp
  sensor_read/imu_sensor.cpp
  sensor_read/kalman_filter.cpp
)
ament_target_dependencies(roscar_imu_publisher
  rclcpp
  shared_interfaces
)
target_link_libraries(roscar_imu_publisher
  ${LIBGPIOD_LIBRARIES}
)

# Ultra Publisher
add_executable(roscar_ultra_publisher
  ros_nodes/roscar_ultra_publisher.cpp
  sensor_read/ultra_sensor.cpp
)
ament_target_dependencies(roscar_ultra_publisher
  rclcpp
  shared_interfaces
)
target_link_libraries(roscar_ultra_publisher
  ${LIBGPIOD_LIBRARIES}
)

# Various Subscribers & Publishers
add_executable(roscar_status_charge_subscriber ros_nodes/roscar_status_charge_subscriber.cpp)
ament_target_dependencies(roscar_status_charge_subscriber rclcpp shared_interfaces)

add_executable(roscar_navigation_goal_subscriber ros_nodes/roscar_navigation_goal_subscriber.cpp)
ament_target_dependencies(roscar_navigation_goal_subscriber rclcpp shared_interfaces)

add_executable(dashboard_status_update_subscriber ros_nodes/dashboard_status_update_subscriber.cpp)
ament_target_dependencies(dashboard_status_update_subscriber rclcpp shared_interfaces)

add_executable(roscar_obstacle_response_subscriber ros_nodes/roscar_obstacle_response_subscriber.cpp)
ament_target_dependencies(roscar_obstacle_response_subscriber rclcpp shared_interfaces)

add_executable(roscar_avoidance_cmd_subscriber ros_nodes/roscar_avoidance_cmd_subscriber.cpp)
ament_target_dependencies(roscar_avoidance_cmd_subscriber rclcpp shared_interfaces)

add_executable(roscar_precision_stop_cmd_subscriber ros_nodes/roscar_precision_stop_cmd_subscriber.cpp)
ament_target_dependencies(roscar_precision_stop_cmd_subscriber rclcpp shared_interfaces)

add_executable(roscar_task_progress_publisher ros_nodes/roscar_task_progress_publisher.cpp)
ament_target_dependencies(roscar_task_progress_publisher rclcpp shared_interfaces)

add_executable(roscar_task_complete_publisher ros_nodes/roscar_task_complete_publisher.cpp)
ament_target_dependencies(roscar_task_complete_publisher rclcpp shared_interfaces)

add_executable(roscar_precision_stop_result_publisher ros_nodes/roscar_precision_stop_result_publisher.cpp)
ament_target_dependencies(roscar_precision_stop_result_publisher rclcpp shared_interfaces)

add_executable(roscar_obstacle_detected_publisher ros_nodes/roscar_obstacle_detected_publisher.cpp)
ament_target_dependencies(roscar_obstacle_detected_publisher rclcpp shared_interfaces)

# Action Servers
add_executable(start_delivery_server action/start_delivery_server.cpp)
ament_target_dependencies(start_delivery_server rclcpp rclcpp_action shared_interfaces)

add_executable(move_to_goal_action_server action/move_to_goal_action_server.cpp)
ament_target_dependencies(move_to_goal_action_server rclcpp rclcpp_action shared_interfaces)

add_executable(scan_inventory_action_server action/scan_inventory_action_server.cpp)
ament_target_dependencies(scan_inventory_action_server rclcpp rclcpp_action shared_interfaces)

add_executable(maintenance_charge_action_server action/maintenance_charge_action_server.cpp)
ament_target_dependencies(maintenance_charge_action_server rclcpp rclcpp_action shared_interfaces)

add_executable(security_patrol_action_server action/security_patrol_action_server.cpp)
ament_target_dependencies(security_patrol_action_server rclcpp rclcpp_action shared_interfaces)

# Control Bridge
add_executable(control_bridge drive/control_bridge.cpp)
ament_target_dependencies(control_bridge
  rclcpp
  geometry_msgs
=======
# ───────────────────────────────────────────────────────────────
# 5. actuator 노드
# ───────────────────────────────────────────────────────────────
add_executable(actuator actuator.cpp)
ament_target_dependencies(actuator
  rclcpp
  std_msgs
)
target_link_libraries(actuator
  ${LIBGPIOD_LIBRARIES}
>>>>>>> Stashed changes
)
# 타겟 링크 필요 시 추가

# ───────────────────────────────────────────────────────────────
# 6. nav_drive 패키지 노드들
# ───────────────────────────────────────────────────────────────
add_executable(astar nav_drive/astar.cpp)
ament_target_dependencies(astar
  rclcpp
  geometry_msgs
  nav_msgs
)

add_executable(waypoint_follower nav_drive/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
)

# ───────────────────────────────────────────────────────────────
# 7. ROS 일반 노드들 (ros_nodes)
# ───────────────────────────────────────────────────────────────
file(GLOB ROS_NODE_SRC
  ros_nodes/*.cpp
)
foreach(src ${ROS_NODE_SRC})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  ament_target_dependencies(${name}
    rclcpp
    std_msgs
    shared_interfaces
  )
endforeach()

# ───────────────────────────────────────────────────────────────
# 8. 센서 읽기용 노드들 (sensor_read)
# ───────────────────────────────────────────────────────────────
file(GLOB SENSOR_READ_SRC
  sensor_read/*.cpp
)
foreach(src ${SENSOR_READ_SRC})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  ament_target_dependencies(${name}
    rclcpp
  )
endforeach()

# ───────────────────────────────────────────────────────────────
# 9. SLLIDAR SDK와 링크
# ───────────────────────────────────────────────────────────────
add_library(sllidar_sdk STATIC ${SLLIDAR_SDK_SRC})
target_include_directories(sllidar_sdk PUBLIC
  sdk/include
  sdk/src
)
ament_target_dependencies(sllidar_sdk
  # 필요한 시스템 라이브러리 없으면 생략
)
# 예시: 내가 만든 Lidar 노드에 링크
# target_link_libraries(roscar_lidar_publisher sllidar_sdk)

# ───────────────────────────────────────────────────────────────
# 10. 설치 지시 (install)
# ───────────────────────────────────────────────────────────────
install(TARGETS
<<<<<<< Updated upstream
  teb_local_planner
  roscar_register_publisher
  roscar_battery_publisher
  sensor_data_publisher
  roscar_presure_reader
  roscar_lidar_publisher
  roscar_imu_publisher
  roscar_ultra_publisher
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
  start_delivery_server
  move_to_goal_action_server
  scan_inventory_action_server
  maintenance_charge_action_server
  security_patrol_action_server
  control_bridge
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
=======
  actuator
  astar
  waypoint_follower
  ${ACTION_SRC}
  ${ROS_NODE_SRC}
  ${SENSOR_READ_SRC}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
>>>>>>> Stashed changes

ament_package()
