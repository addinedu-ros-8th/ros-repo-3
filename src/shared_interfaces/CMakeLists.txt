cmake_minimum_required(VERSION 3.5)
project(shared_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStatus.msg"
  "msg/Emergency.msg"
  "msg/TaskResult.msg"
  "srv/ManualOverride.srv"
)

ament_package()