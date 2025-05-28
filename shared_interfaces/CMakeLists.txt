cmake_minimum_required(VERSION 3.8)
project(shared_interfaces)

# Compiler flags for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)      

# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/BatteryStatus.msg"
  "msg/LogEvent.msg"
  "msg/ObstacleAvoidanceCmd.msg"
  "msg/ObstacleResponse.msg"
  "msg/PrecisionStopResult.msg"
  "msg/RoscarPoseUpdate.msg"
  "msg/RoscarRegister.msg"
  "msg/RoscarStatusLog.msg"
  "msg/StartTask.msg"
  "msg/TaskComplete.msg"
  "msg/TaskProgress.msg"
  "msg/ImuStatus.msg"
  "msg/LidarScan.msg"
  "msg/UltraStatus.msg"
  "msg/RoscarStatus.msg"

  # Services
  "srv/LogQuery.srv"
  "srv/Login.srv"
  "srv/QueryRoscarStatus.srv"

  # Actions
  "action/MoveToGoal.action"
  "action/StartDelivery.action"

  DEPENDENCIES
    builtin_interfaces
    action_msgs
    geometry_msgs                      
)

# Enable lint tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
