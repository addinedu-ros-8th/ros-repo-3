cmake_minimum_required(VERSION 3.8)
project(shared_interfaces)

# Compiler flags for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/AccessResult.msg"
  "msg/AIModuleStatus.msg"
  "msg/BatteryStatus.msg"
  "msg/ChargeCommand.msg"
  "msg/DashboardStatus.msg"
  "msg/EmergencyAlert.msg"
  "msg/EmergencyCommand.msg"
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
  "msg/SensorData.msg"
  "msg/SensorFusion.msg"
  "msg/TaskComplete.msg"
  "msg/TaskProgress.msg"

  # Services
  "srv/ManualOverride.srv"
  "srv/QueryTaskStatus.srv"
  "srv/RequestDiagnostics.srv"
  "srv/StartInventoryCheck.srv"

  # Actions
  "action/ChargeProcess.action"
  "action/MaintenanceCharge.action"
  "action/MoveToGoal.action"
  "action/PatrolZone.action"
  "action/ScanInventory.action"
  "action/SecurityPatrol.action"
  "action/StartDelivery.action"

  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export
ament_export_dependencies(rosidl_default_runtime)

ament_package()
