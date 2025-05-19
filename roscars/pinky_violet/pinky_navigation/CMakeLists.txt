cmake_minimum_required(VERSION 3.8)
project(pinky_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# Nav2 static map server & lifecycle manager
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

# Cartographer occupancy grid node
find_package(cartographer_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  pinky_navigation/initialpose.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 스케너리 무시 설정
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch params map rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ament_cmake_python
  rclcpp
  nav2_map_server
  nav2_lifecycle_manager
  cartographer_ros
)

ament_package()
