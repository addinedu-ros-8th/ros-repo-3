cmake_minimum_required(VERSION 3.5)
project(shared_interfaces)

# 빌드 도구 및 메시지 생성 관련 패키지
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 타입에 필요한 의존 패키지
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 메시지 및 서비스 정의
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStatus.msg"
  "msg/Emergency.msg"
  "msg/TaskResult.msg"
  "msg/RobotRegister.msg"
  "msg/LidarScan.msg"
  "msg/ImuStatus.msg"
  "msg/UltraStatus.msg"
  "msg/RobotInfo.msg"
  "msg/AccessResult.msg"
  "srv/ManualOverride.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Python 타입 서포트 및 CPP 타입 서포트 지정 (Python 바인딩 생성을 위해 필요)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(python_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_python")

ament_export_dependencies(rosidl_default_runtime)

# 패키지 설치 정의
ament_package()
