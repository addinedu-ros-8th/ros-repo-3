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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# JSONCPP (LiDAR JSON 변환용)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBJSONCPP REQUIRED jsoncpp)

# libgpiod 설정 (초음파 센서 인터페이스)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# ───────────────────────────────────────────────────────────────
# 2. include 디렉토리 설정 (글로벌)
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
file(GLOB_RECURSE SLLIDAR_SDK_SRC
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
)
add_library(sllidar_sdk STATIC ${SLLIDAR_SDK_SRC})
target_include_directories(sllidar_sdk PUBLIC
  sdk/include
  sdk/src
  sdk/src/arch/linux
  sdk/src/dataunpacker
)
ament_target_dependencies(sllidar_sdk
  PkgConfig
)
target_link_libraries(sllidar_sdk
  ${LIBJSONCPP_LIBRARIES}
  ${LIBGPIOD_LIBRARIES}
)

# ───────────────────────────────────────────────────────────────
# 4. sensor_read 라이브러리
# ───────────────────────────────────────────────────────────────
add_library(sensor_read_lib
  sensor_read/battery.cpp
  sensor_read/imu_sensor.cpp
  sensor_read/ultra_sensor.cpp
  sensor_read/kalman_filter.cpp
)
ament_target_dependencies(sensor_read_lib
  rclcpp
)
target_include_directories(sensor_read_lib PUBLIC
  include/mobile_controller
  ${LIBGPIOD_INCLUDE_DIRS}        # libgpiod 헤더
)
target_link_libraries(sensor_read_lib
  ${LIBGPIOD_LIBRARIES}           # libgpiod 라이브러리
)

# ───────────────────────────────────────────────────────────────
# 5. nav_drive 라이브러리
# ───────────────────────────────────────────────────────────────
add_library(nav_drive_lib
  nav_drive/astar.cpp
  nav_drive/waypoint_follower.cpp
)
ament_target_dependencies(nav_drive_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
)
target_include_directories(nav_drive_lib PUBLIC include/mobile_controller)
target_link_libraries(nav_drive_lib
  rclcpp_action::rclcpp_action
)

 # ───────────────────────────────────────────────────────────────
 # 5. nav_drive 라이브러리
 # ───────────────────────────────────────────────────────────────
 

# ───────────────────────────────────────────────────────────────
# 6. waypoint_follower 실행 파일
# ───────────────────────────────────────────────────────────────
add_executable(waypoint_follower
  nav_drive/waypoint_follower.cpp
)
ament_target_dependencies(waypoint_follower
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
)
target_link_libraries(waypoint_follower
  nav_drive_lib
)
install(TARGETS
  waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

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
  sllidar_sdk
)
target_include_directories(teb_local_planner PRIVATE include/mobile_controller)

add_executable(control_bridge drive/control_bridge.cpp)
ament_target_dependencies(control_bridge
  rclcpp
  geometry_msgs
)
target_link_libraries(control_bridge
  nav_drive_lib
)
target_include_directories(control_bridge PRIVATE include/mobile_controller)

# ───────────────────────────────────────────────────────────────
# 7. sensor_data_publisher (JsonCpp 포함)
# ───────────────────────────────────────────────────────────────



# ───────────────────────────────────────────────────────────────
# 8. 개별 실행 노드 정의 (나머지)
# ───────────────────────────────────────────────────────────────
macro(define_node name)
  add_executable(${name} ros_nodes/${name}.cpp)
  ament_target_dependencies(${name}
    rclcpp
    shared_interfaces
  )
  target_include_directories(${name} PRIVATE include/mobile_controller)
endmacro()

define_node(roscar_battery_publisher)
target_link_libraries(roscar_battery_publisher sensor_read_lib)

define_node(roscar_imu_publisher)
target_link_libraries(roscar_imu_publisher sensor_read_lib)

define_node(roscar_ultra_publisher)
target_link_libraries(roscar_ultra_publisher sensor_read_lib)

define_node(roscar_lidar_publisher)
target_link_libraries(roscar_lidar_publisher sllidar_sdk)

# ───────────────────────────────────────────────────────────────
# 9. ros_nodes 자동으로 나머지 노드 추가
# ───────────────────────────────────────────────────────────────
file(GLOB OTHER_SRCS "ros_nodes/*.cpp")
foreach(src IN LISTS OTHER_SRCS)
  get_filename_component(name ${src} NAME_WE)
  if(NOT TARGET ${name})
    add_executable(${name} ${src})
    ament_target_dependencies(${name}
      rclcpp
      std_msgs
      shared_interfaces
    )
    target_include_directories(${name} PRIVATE include/mobile_controller)
  endif()
endforeach()

# ───────────────────────────────────────────────────────────────
# 10. 설치 지시
# ───────────────────────────────────────────────────────────────
install(TARGETS
  teb_local_planner
  control_bridge
  nav_drive_lib
  roscar_battery_publisher
  roscar_imu_publisher
  roscar_ultra_publisher
  roscar_lidar_publisher
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
