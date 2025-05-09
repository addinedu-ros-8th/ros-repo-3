cmake_minimum_required(VERSION 3.5)
project(shared_interfaces)

# 빌드 도구 및 메시지 생성 관련 패키지
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 타입에 필요한 의존 패키지
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(msg_files
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
  "msg/LogEvent.msg"
  "msg/LogEventAdmin.msg"
  "msg/NavigationGoal.msg"
  "msg/ObstacleAvoidanceCmd.msg"
  "msg/ObstacleDetected.msg"
  "msg/ObstacleResponse.msg"
  "msg/PrecisionStopCmd.msg"
  "msg/PrecisionStopResult.msg"
  "msg/QRCodeScan.msg"
  "msg/RoscarPoseUpdate.msg"
  "msg/RoscarStatusLog.msg"
  "msg/RoscarInfo.msg"
  "msg/RoscarRegister.msg"
  "msg/SensorFusion.msg"
  "msg/TaskComplete.msg"
  "msg/TaskProgress.msg"
  "msg/UltraStatus.msg"
)

set(srv_files
  "srv/ManualOverride.srv"
  "srv/QueryTaskStatus.srv"
  "srv/RequestDiagnostics.srv"
  "srv/StartInventoryCheck.srv"
)

set(action_files
  "action/ChargeProcess.action"
  "action/MaintenanceCharge.action"
  "action/MoveToGoal.action"
  "action/PatrolZone.action"
  "action/ScanInventory.action"
  "action/SecurityPatrol.action"
  "action/StartDelivery.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Python 타입 서포트 및 CPP 타입 서포트 지정 (Python 바인딩 생성을 위해 필요)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(python_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_python")

ament_export_dependencies(rosidl_default_runtime)

# 패키지 설치 정의
ament_package()
