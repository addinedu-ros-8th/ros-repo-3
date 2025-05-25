cmake_minimum_required(VERSION 3.8)
project(cart_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(micro_ros_agent REQUIRED)

# 실제 실행 파일 등록
add_executable(cart_controller_node shared_interface/src/cart_controller_node.cpp)

# 종속성 연결
ament_target_dependencies(cart_controller_node
  rclcpp
  micro_ros_agent
)

# 설치 경로 설정
install(TARGETS
  cart_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
