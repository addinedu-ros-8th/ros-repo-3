cmake_minimum_required(VERSION 3.8)
project(pinky_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_python REQUIRED)

# # Python node
# ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS
#   pinky_navigation/initialpose.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# C++ node
add_executable(initialpose_handler pinky_navigation/src/initialpose_handler.cpp)
ament_target_dependencies(initialpose_handler
  rclcpp
  geometry_msgs
  cartographer_ros_msgs
  ament_index_cpp
)
install(TARGETS
  initialpose_handler
  DESTINATION lib/${PROJECT_NAME}
)

# Shared resources
install(
  DIRECTORY launch params map rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
