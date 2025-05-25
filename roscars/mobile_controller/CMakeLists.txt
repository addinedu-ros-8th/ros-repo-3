cmake_minimum_required(VERSION 3.5)
project(mobile_controller)

# ───────────────────────────────────────────────────────────────
# 매크로 정의: define_node(name)
# ───────────────────────────────────────────────────────────────
macro(define_node name)
  add_executable(${name} ros_nodes/${name}.cpp)
  ament_target_dependencies(${name}
    rclcpp
    shared_interfaces
  )
  target_include_directories(${name} PRIVATE include/mobile_controller)
endmacro()

# ───────────────────────────────────────────────────────────────
# 1. 패키지 설정
# ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBJSONCPP REQUIRED jsoncpp)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# ───────────────────────────────────────────────────────────────
# 2. include 설정
# ───────────────────────────────────────────────────────────────
include_directories(
  include
  include/mobile_controller
  sdk/include
  sdk/src
  sdk/src/arch/linux
  sdk/src/dataunpacker
)

# ───────────────────────────────────────────────────────────────
# 3. SLLIDAR SDK 라이브러리
# ───────────────────────────────────────────────────────────────
set(SLLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
file(GLOB_RECURSE SLLIDAR_SDK_SRC "${SLLIDAR_SDK_PATH}/src/*.cpp")

add_library(sllidar_sdk STATIC ${SLLIDAR_SDK_SRC})
target_include_directories(sllidar_sdk PUBLIC
  sdk/include
  sdk/src
  sdk/src/arch/linux
  sdk/src/dataunpacker
)
ament_target_dependencies(sllidar_sdk PkgConfig)
target_link_libraries(sllidar_sdk ${LIBJSONCPP_LIBRARIES} ${LIBGPIOD_LIBRARIES})

# ───────────────────────────────────────────────────────────────
# 4. sensor_read_lib
# ───────────────────────────────────────────────────────────────
add_library(sensor_read_lib
  sensor_read/battery.cpp
  sensor_read/imu_sensor.cpp
  sensor_read/ultra_sensor.cpp
  sensor_read/kalman_filter.cpp
)
ament_target_dependencies(sensor_read_lib rclcpp)
target_include_directories(sensor_read_lib PUBLIC
  include/mobile_controller
  ${LIBGPIOD_INCLUDE_DIRS}
)

# ───────────────────────────────────────────────────────────────
# 5. 실행 노드
# ───────────────────────────────────────────────────────────────

# 구독자 노드
add_executable(roscar_obstacle_response_subscriber ros_nodes/roscar_obstacle_response_subscriber.cpp)
ament_target_dependencies(roscar_obstacle_response_subscriber rclcpp shared_interfaces)

add_executable(roscar_avoidance_cmd_subscriber ros_nodes/roscar_avoidance_cmd_subscriber.cpp)
ament_target_dependencies(roscar_avoidance_cmd_subscriber rclcpp shared_interfaces)

# 퍼블리셔 노드
add_executable(roscar_task_progress_publisher ros_nodes/roscar_task_progress_publisher.cpp)
ament_target_dependencies(roscar_task_progress_publisher rclcpp shared_interfaces)

add_executable(roscar_task_complete_publisher ros_nodes/roscar_task_complete_publisher.cpp)
ament_target_dependencies(roscar_task_complete_publisher rclcpp shared_interfaces)

add_executable(roscar_precision_stop_result_publisher ros_nodes/roscar_precision_stop_result_publisher.cpp)
ament_target_dependencies(roscar_precision_stop_result_publisher rclcpp shared_interfaces)

add_executable(roscar_ssid_publisher ros_nodes/roscar_ssid_publisher.cpp)
ament_target_dependencies(roscar_ssid_publisher rclcpp std_msgs)

add_executable(roscar_lidar_publisher ros_nodes/roscar_lidar_publisher.cpp ${SLLIDAR_SDK_SRC})
ament_target_dependencies(roscar_lidar_publisher rclcpp shared_interfaces)
target_compile_definitions(roscar_lidar_publisher PRIVATE SDK_PLATFORM_LINUX)
target_link_libraries(roscar_lidar_publisher sllidar_sdk)

add_executable(roscar_ultra_publisher ros_nodes/roscar_ultra_publisher.cpp sensor_read/ultra_sensor.cpp)
ament_target_dependencies(roscar_ultra_publisher rclcpp shared_interfaces)
target_link_libraries(roscar_ultra_publisher ${LIBGPIOD_LIBRARIES} sensor_read_lib)

add_executable(roscar_imu_publisher ros_nodes/roscar_imu_publisher.cpp)
ament_target_dependencies(roscar_imu_publisher rclcpp shared_interfaces)
target_link_libraries(roscar_imu_publisher sensor_read_lib)

add_executable(roscar_register_publisher ros_nodes/roscar_register_publisher.cpp)
ament_target_dependencies(roscar_register_publisher rclcpp shared_interfaces)

add_executable(roscar_presure_subscriber ros_nodes/roscar_presure_subscriber.cpp)
ament_target_dependencies(roscar_presure_subscriber rclcpp shared_interfaces)

add_executable(roscar_battery_publisher ros_nodes/roscar_battery_publisher.cpp)
ament_target_dependencies(roscar_battery_publisher rclcpp shared_interfaces)
target_link_libraries(roscar_battery_publisher sensor_read_lib)

# 액션 서버
add_executable(start_delivery_server action/start_delivery_server.cpp)
ament_target_dependencies(start_delivery_server rclcpp rclcpp_action shared_interfaces)

# nav_drive
add_library(nav_drive_lib
  nav_drive/astar.cpp
  nav_drive/waypoint_follower.cpp
)
ament_target_dependencies(nav_drive_lib rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs)
target_include_directories(nav_drive_lib PUBLIC include/mobile_controller)
target_link_libraries(nav_drive_lib rclcpp_action::rclcpp_action)

add_executable(waypoint_follower nav_drive/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs)
target_link_libraries(waypoint_follower nav_drive_lib)

add_executable(teb_local_planner drive/teb_local_planner.cpp)
ament_target_dependencies(teb_local_planner
  rclcpp sensor_msgs nav_msgs geometry_msgs shared_interfaces
  tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(teb_local_planner sllidar_sdk)
target_include_directories(teb_local_planner PRIVATE include/mobile_controller)

add_executable(control_bridge drive/control_bridge.cpp)
ament_target_dependencies(control_bridge rclcpp geometry_msgs)
target_link_libraries(control_bridge nav_drive_lib)
target_include_directories(control_bridge PRIVATE include/mobile_controller)

# ───────────────────────────────────────────────────────────────
# 설치
# ───────────────────────────────────────────────────────────────
install(TARGETS
  roscar_register_publisher
  roscar_presure_subscriber
  roscar_battery_publisher
  roscar_lidar_publisher
  roscar_imu_publisher
  roscar_ultra_publisher
  roscar_obstacle_response_subscriber
  roscar_avoidance_cmd_subscriber
  roscar_task_progress_publisher
  roscar_task_complete_publisher
  roscar_precision_stop_result_publisher
  roscar_ssid_publisher

  start_delivery_server
  waypoint_follower
  teb_local_planner
  control_bridge

  nav_drive_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
