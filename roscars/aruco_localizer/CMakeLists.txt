cmake_minimum_required(VERSION 3.5)
project(aruco_localizer)

# Python 인터프리터 찾기
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Python install directory 설정
ament_python_install_package(${PROJECT_NAME})

# launch 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 노드 파일 설치
install(PROGRAMS
  aruco_localizer/aruco_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
