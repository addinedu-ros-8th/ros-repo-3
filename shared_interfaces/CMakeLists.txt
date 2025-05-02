cmake_minimum_required(VERSION 3.5)
project(shared_interfaces)

# 메시지/서비스 생성 도구
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 메시지 정의
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStatus.msg"
  "msg/Emergency.msg"
  "msg/TaskResult.msg"
  "msg/RobotStatus.msg"  # 추가한 RobotStatus 메시지
  "msg/LidarScan.msg"
  "msg/ImuStatus.msg"
  "srv/ManualOverride.srv"
  DEPENDENCIES std_msgs
)

ament_package()
