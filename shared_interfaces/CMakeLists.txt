cmake_minimum_required(VERSION 3.8)
project(shared_interfaces)

# Compiler flags for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)   # std_msgs 의존성 추가
find_package(geometry_msgs REQUIRED)  # geometry_msgs 의존성 추가

# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AccessResult.msg"
  "msg/AIModuleStatus.msg"
  "msg/BatteryStatus.msg"
  "msg/ChargeCommand.msg"
  "msg/DashboardStatus.msg"
  "msg/EmergencyAlert.msg"
  "msg/EmergencyCommand.msg"
  "msg/ImuStatus.msg"
  "msg/InventoryResult.msg"
  "msg/LidarScan.msg"
  "msg/LogEventAdmin.msg"
  "msg/LogEvent.msg"
  "msg/NavigationGoal.msg"
  "msg/ObstacleAvoidanceCmd.msg"
  "msg/ObstacleDetected.msg"
  "msg/ObstacleResponse.msg"
  "msg/PrecisionStopCmd.msg"
  "msg/PrecisionStopResult.msg"
  "msg/QRCodeScan.msg"
  "msg/RoscarInfo.msg"
  "msg/RoscarPoseUpdate.msg"
  "msg/RoscarRegister.msg"
  "msg/RoscarStatusLog.msg"
  "msg/SensorFusion.msg"
  "msg/TaskComplete.msg"
  "msg/TaskProgress.msg"
  "msg/UltraStatus.msg"
  "srv/ManualOverride.srv"
  "srv/QueryTaskStatus.srv"
  "srv/RequestDiagnostics.srv"
  "srv/StartInventoryCheck.srv"
  "action/ChargeProcess.action"
  "action/MaintenanceCharge.action"
  "action/MoveToGoal.action"
  "action/PatrolZone.action"
  "action/ScanInventory.action"
  "action/SecurityPatrol.action"
  "action/StartDelivery.action"
  DEPENDENCIES std_msgs geometry_msgs  # 의존성 추가
)

# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
