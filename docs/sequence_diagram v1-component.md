# Task Request

```plantuml
@startuml
actor User
participant "Staff GUI" as Staff
participant "Main Service" as Main
participant "Mobile Controller" as Mobile

User -> Staff : 재고 검색
activate Staff

Staff -> Main : 재고 검색\n(모델명, 사이즈)
activate Main
Main -> Main : DB 재고 정보 검색
deactivate Main
Main -> Staff : 재고 검색 결과 전송\n(모델명, 사이즈, 수량, 위치)
deactivate Staff

Staff -> Main : 재고 요청
activate Main
Main -> Main : DB 재고, 위치 검색

Main -> Mobile : 픽업 요청\n(모델명, 사이즈, 수량) x N
activate Mobile
Mobile -> Mobile : 제품 집재 및 운송
Mobile -> Main : 작업 결과 전송
deactivate Mobile

Main -> Staff : 작업 결과 전송
deactivate Main
@enduml
```


# Staff PC

```plantuml
@startuml
actor User
participant "Staff GUI" as Staff
participant "Main Service" as Main
participant "Mobile Controller" as Mobile

User -> Staff : 프로그램 실행
activate Staff
Staff -> Main : 서버 연결 시도
activate Main
Main --> Staff : 서버 연결 결과\n(result)
deactivate Main

Staff -> Main : 접속 요청\n(ID, password)
activate Main
Main -> Main : DB 회원 정보 검색
Main --> Staff : 로그인 결과\n(result)
deactivate Main

Staff -> Main : 재고 요청
activate Main
Main -> Main : 상품 ID 조회
Main -> Main : DB Item 정보 검색
Main --> Staff : 재고 응답 (있음)\n[모델명, 색상, 사이즈, 개수, 위치]
deactivate Main
Staff -> Staff : GUI에 상품정보 표시

Staff -> Staff : QR 스캔
Staff -> Staff : 버튼 선택

Staff -> Main : [요청] 요청 전송
activate Main
Main -> Mobile : 픽업 이동 명령
activate Mobile
Mobile -> Mobile : 상태: 요청됨
deactivate Mobile

Staff -> Staff : 팝업 표시: 요청됨
Staff -> Staff : 카메라 화면 전환

Mobile -> Mobile : 실제 이동 수행
Mobile --> Main : 도착 알림
Main --> Staff : 상태: 배송중 표시
deactivate Main

Staff -> Staff : 도착 상태 표시

Staff -> Staff : 픽업 완료 + 담기버튼
Staff -> Main : 요청 완료 전송
activate Main
Main -> Mobile : 핑거 복귀
deactivate Main

Staff -> Staff : 담기
Staff -> Staff : 담기 캐시 저장
Staff -> Staff : 팝업: 담아짐
Staff -> Staff : 카메라 전환
Staff -> Staff : 담은목록 표시
Staff -> Staff : [담은목록] 캐시 로딩
Staff -> Staff : 목록 표시

Staff -> Staff : 담기 취소
Staff -> Staff : 항목 삭제

Staff -> Main : 요청
activate Main
Main -> Mobile : 일괄 요청 전송
Mobile -> Mobile : 요청됨 / 배송중
deactivate Main

Staff -> Staff : 반품등록
Staff -> Main : 재고 업데이트
activate Main
Main -> Mobile : 랙 위치 이동+스캔
deactivate Main
@enduml
```


## Login Flow

```plantuml
@startuml
actor User
participant "Staff GUI" as Staff
participant "Main Service" as Main

User -> Staff : 프로그램 실행
activate Staff
Staff -> Main : 서버 연결 시도
activate Main
Main --> Staff : 서버 연결 결과\n(result)
deactivate Main

Staff -> Main : 로그인 시도\n(ID, password)
activate Main
Main -> Main : DB 회원 정보 검색
Main --> Staff : 로그인 결과\n(result)
deactivate Main
deactivate Staff
@enduml

```



## 재고 요청 Flow

```plantuml
@startuml
actor User
participant "Staff GUI" as Staff
participant "Main Service" as Main
participant "Mobile Controller" as Mobile

User -> Staff : QR 스캔
activate Staff
Staff -> Main : 상품 ID 조회
activate Main
Main -> Main : DB Item 정보 검색
Main --> Staff : 재고 응답 (있음)\n(모델명, 색상, 사이즈, 개수, 위치)
deactivate Main

Staff -> Staff : GUI에 상품정보 표시
Staff -> Staff : 버튼 선택

Staff -> Main : [요청] 요청 전송
activate Main
Main -> Mobile : 픽기 이동 명령
activate Mobile
Mobile -> Mobile : 상태: 요청됨
deactivate Mobile
deactivate Main

Staff -> Staff : 팝업 표시: 요청됨
Staff -> Staff : 카메라 화면 전환

Mobile -> Mobile : 실제 이동 수행
Mobile --> Main : 도착 알림
activate Main
Main --> Staff : 상태: 배송중 표시
deactivate Main

Staff -> Staff : 도착 상태 표시

== 픽업 및 담기 ==

Staff -> Staff : 픽업 완료 + 담기버튼
Staff -> Main : 요청 완료 전송
activate Main
Main -> Mobile : 픽기 복귀
deactivate Main

Staff -> Staff : 담기
Staff -> Staff : 담기 캐시 지정
Staff -> Staff : 팝업: 담아짐
Staff -> Staff : 카메라 전환
Staff -> Staff : 담은목록 표시
Staff -> Staff : [담은목록] 캐시 로딩
Staff -> Staff : 목록 표시

Staff -> Staff : 담기 취소
Staff -> Staff : 항목 삭제

== 일괄 요청 및 반품 ==

Staff -> Main : 요청
activate Main
Main -> Mobile : 일괄 요청 전송
Mobile -> Mobile : 요청됨 / 배송중
deactivate Main

Staff -> Staff : 반품등록
Staff -> Main : 재고 업데이트
activate Main
Main -> Mobile : 랙 위치 이동+스캔
deactivate Main
@enduml

```





# Manager PC

## Log 확인

```plantuml
@startuml
actor User
participant "Manager GUI" as Manager
participant "Main Service" as Main

User -> Manager : 로그 검색
activate Manager

Manager -> Main : 로그 검색\n[재고, 출고내역,\n로봇 이벤트,\n작업 로그(수신, 취소, 완료)]
activate Main
Main -> Main : DB 로그 조회
Main --> Manager : 로그 결과\n[재고, 출고내역,\n로봇 이벤트, 작업 로그]
deactivate Main
deactivate Manager
@enduml

```



### 재고 정보 확인

```plantuml
@startuml
actor User
participant "Manager GUI" as Manager
participant "Main Service" as Main

User -> Manager : 제품 조회
activate Manager

Manager -> Main : 제품 조회\n(Item ID)
activate Main
Main -> Main : 제품 DB 조회\n(Item ID)
Main --> Manager : 제품 조회 결과\n[Item ID, Storage Location,\narUco ID, Stock Quantity,\nx_coord, y_coord, z_coord,\nlast_inventory_update]
deactivate Main
deactivate Manager
@enduml

```




### 로봇 이벤트 조회

```plantuml
@startuml
actor User
participant "Manager GUI" as Manager
participant "Main Service" as Main

User -> Manager : 로봇 이벤트 조회
activate Manager

Manager -> Main : 로봇 이벤트 조회\n(event_id)
activate Main
Main -> Main : DB 로봇 이벤트 조회\n(event_id)
Main --> Manager : 로봇 이벤트 조회\n[event_id, robot_id,\ntask_id, event_type,\nevent_timestamp]
deactivate Main
deactivate Manager
@endum
```


### 작업 로그 확인

```plantuml
@startuml
actor User
participant "Manager GUI" as Manager
participant "Main Service" as Main

User -> Manager : 작업 로그 조회
activate Manager

Manager -> Main : 작업 로그 조회\n[Task_id]
activate Main
Main -> Main : 작업 로그 DB 조회
Main --> Manager : 작업 로그 조회\n[Task_id, Robot_id, Function_id,\nUser_id, Task_Label, Task_Status,\nTask_priority_level, Task_start_time,\nTask_end_time, Trigger_source]
deactivate Main
deactivate Manager
@enduml
```


## 로봇 동작 상태 확인

```plantuml
@startuml
actor User
participant "Manager GUI" as Manager
participant "Main Service" as Main
participant "Mobile Controller" as Mobile

User -> Manager : 로봇 상태 확인
activate Manager

Manager -> Main : 로봇 조회\n(Robot ID)
activate Main

loop
    Main -> Mobile : 로봇 상태 송신\n[Robot ID, Battery 잔량,\n동작상태(Standby, Driving,\nCharging, Error, EMS, Disconnect),\nBasket Status(True, False)]
    activate Mobile
    Mobile -> Main : DB 로봇 상태 업데이트
    deactivate Mobile
end

deactivate Main
deactivate Manager
@enduml
```



## 로봇 등록 삭제

```plantuml
@startuml
actor User
participant "Manager GUI" as Manager
participant "Main Service" as Main

== 로봇 등록 ==

User -> Manager : 로봇 등록
activate Manager
Manager -> Main : 로봇 이름 전송\n[Robot Name(고유)]
activate Main
Main -> Main : DB 저장
Main --> Manager : 로봇 등록 완료
deactivate Main
deactivate Manager

== 로봇 삭제 ==

User -> Manager : 로봇 삭제
activate Manager
Manager -> Main : 로봇 선택\n[Robot ID]
activate Main
Main -> Main : DB 저장
Main --> Manager : 로봇 삭제 완료
deactivate Main
deactivate Manager
@enduml
```


## Robot Location

```plantuml
@startuml
actor User
participant "Manager GUI" as Manager
participant "Main Service" as Main
participant "Mobile Controller" as Mobile

User -> Manager : 로봇 위치 확인
activate Manager

loop
    Manager -> Main : 로봇 위치 확인
    activate Main
    Main -> Mobile : 로봇 위치 확인
    activate Mobile
    Mobile --> Main : 로봇 위치 전송\n[x_coord, y_coord, z_coord]
    deactivate Mobile
    deactivate Main
end

deactivate Manager
@enduml
```



## 재고 파악 모드

```plantuml
@startuml
actor Actor
participant "Manager PC" as Manager
participant "Main Service" as Main
participant "Mobile Controller" as Mobile

Actor -> Manager : 재고 파악 요청
activate Manager
Manager -> Main : 재고 파악 모드 실행
activate Main
Main -> Mobile : 재고 파악 모드 실행
activate Mobile

Mobile -> Mobile : 재고 파악 진행\n[제품 QR 탐지]
Mobile -> Mobile : 재고 DB Update
Mobile --> Main : 재고 정보 전송
deactivate Mobile

Main --> Manager : 재고 정보 전송
deactivate Main
deactivate Manager
@enduml
```

# Main Service

## Task Controller Flow

```plantuml
@startuml
participant StaffPC
participant MainService
database Database
participant MobileController

== Task 생성 요청 ==

activate StaffPC
StaffPC -> MainService: CreateTaskRequest (pickup_location, dropoff_location, item_id, priority)

activate MainService
MainService -> MainService: Create new Task object (status = PENDING)

MainService -> Database: InsertTask (task info)

== 대기 로봇 검색 및 선택 ==

activate Database
MainService -> Database: QueryIdleRobots (status=IDLE)
Database --> MainService: RobotListResult (robot_id, location, battery_level)
MainService -> MainService: SelectOptimalRobot (best robot)

== 작업 할당 ==

MainService -> MobileController: AssignTask (task_id, pickup_location, dropoff_location, item_id)

== 작업 수락 or 거부 ==

activate MobileController
MobileController -> MainService: TaskAck (task_id, result, reason)

alt Task Assignment Accepted
    MainService -> MainService: UpdateTaskStatus (IN_PROGRESS)
else Task Assignment Rejected
    MainService -> MainService: TryNextAvailableRobot
    alt No More Robots Available
        MainService -> MainService: QueueTask
    end
end

== 작업 진행 업데이트 ==

MobileController -> MainService: TaskProgressUpdate (task_id, current_location, progress_detail)
MainService -> Database: UpdateTask_Progress (progress_detail, location)

== 작업 완료 ==

MobileController -> MainService: TaskCompleteReport (task_id, result=SUCCESS)
MainService -> Database: UpdateTask_Complete (COMPLETED)
MainService -> StaffPC: CreateTaskResponse (SUCCESS)

== 작업 실패 ==

MobileController -> MainService: TaskCompleteReport (task_id, result=FAILURE)
MainService -> Database: UpdateTask_Complete (FAILED)
MainService -> StaffPC: CreateTaskResponse (FAILURE)

== 작업 타임아웃 처리 ==

MainService -> MainService: Detect task timeout
MainService -> Database: UpdateTask_Status (FAILED)
MainService -> StaffPC: CreateTaskResponse (Timeout Failure)

== 사용자에 의한 작업 취소 ==

StaffPC -> MainService: CancelTaskRequest (task_id)
MainService -> MobileController: CancelAssignedTask (task_id)
MobileController -> MainService: CancelAck (task_id)
MainService -> Database: UpdateTask_Status (CANCELLED)
deactivate Database

MainService -> StaffPC: CancelTaskResponse (SUCCESS)
deactivate StaffPC

== 대기 로봇 생긴 경우 대기 작업 할당 ==

MobileController -> MainService: IdleStatusReport (robot_id)
deactivate MobileController

MainService -> MainService: AssignQueuedTaskIfAvailable (if any)

@enduml

deactivate MainService
```

## Received Request and Respond to Staff GUI

```plantuml
@startuml
participant StaffGUI
participant MainService
database Database
participant MobileController

== 로그인 플로우 ==

StaffGUI -> MainService: LoginRequest (name, password)
activate StaffGUI
activate MainService
MainService -> Database: SearchUserByName (name)
activate Database
Database --> MainService: UserSearchResult (id, name, role, can_call_robot, password)
deactivate Database
MainService -> MainService: Verify password
alt Password Match
    MainService -> Database: SearchGUIDatas (user-specific)
    activate Database
    Database --> MainService: GUIDatasResult
    deactivate Database
    MainService -> StaffGUI: LoginResponse (success, GUI data)
else Password Mismatch
    MainService -> StaffGUI: LoginResponse (failure)
    deactivate MainService
    deactivate StaffGUI
end

== 상품 조회 플로우 ==

StaffGUI -> MainService: SearchQRCodeData (QR_code)
activate StaffGUI
activate MainService
MainService -> Database: SearchQRCode (QR_code)
activate Database
Database --> MainService: QRCodeResult (QR_id, QR_code, shoes_id)
deactivate Database
alt QR Code Found
    MainService -> Database: SearchShoesData (shoes_id)
    activate Database
    Database --> MainService: ShoesDataResult (shoes_id, shoes_name, shoes_size, shoes_color, shoes_quantity, shoes_location)
    deactivate Database
    alt Shoes Data Found
        MainService -> StaffGUI: ShoesInfoResponse (shoes information)
    else Shoes Data Not Found
        MainService -> StaffGUI: ShoesInfoSearchFailed (reason="Shoes data missing")
    end
else QR Code Not Found
    MainService -> StaffGUI: QRCodeSearchFailed (reason="QR not found")
    deactivate MainService
    deactivate StaffGUI
end

== 로봇 상태 조회 및 작업 요청 플로우 ==

StaffGUI -> MainService: RequestRobotStatus
activate StaffGUI
activate MainService
MainService -> Database: SearchRobotStatus
activate Database
Database --> MainService: RobotStatusResult
deactivate Database
alt Robot Status Found
    MainService -> MobileController: CheckAvailableRobot
    activate MobileController
    alt Available Robot Found
        MainService -> MobileController: SendNavigationGoal (/robot/navigation/goal)
        deactivate MobileController

        MainService -> Database: RecordTaskStatus (task creation and status)
        activate Database
        deactivate Database
    else No Available Robot
        MainService -> StaffGUI: RobotAssignFailed (reason="No available robot")
    end
else Robot Status Not Found
    MainService -> StaffGUI: RobotStatusSearchFailed (reason="Robot offline")
    deactivate MainService
    deactivate StaffGUI
end

@enduml
```

## Received Request and Respond to Manager GUI

```plantuml
@startuml
participant ManagerGUI
participant MainService
database Database

== 로그인 플로우 ==

ManagerGUI -> MainService: ManagerLoginRequest (name, password)
activate ManagerGUI
activate MainService
MainService -> Database: SearchUserByName (name)
activate Database
Database --> MainService: UserSearchResult (id, name, role, permissions, password)
deactivate Database
MainService -> MainService: Verify password
alt Password Match
    MainService -> Database: SearchAdminDatas (초기 대시보드 데이터)
    activate Database
    Database --> MainService: AdminDatasResult (dashboard data)
    deactivate Database
    MainService -> ManagerGUI: ManagerLoginResponse (success, dashboard data)
else Password Mismatch
    MainService -> ManagerGUI: ManagerLoginResponse (failure)
    deactivate MainService
    deactivate ManagerGUI
end

== 재고 정보 조회 ==

ManagerGUI -> MainService: RequestInventoryData
activate ManagerGUI
activate MainService
MainService -> Database: SearchInventoryData (optional filter)
activate Database
Database --> MainService: InventoryDataResult (inventory list)
deactivate Database
MainService -> ManagerGUI: InventoryDataResponse (inventory list)
deactivate MainService
deactivate ManagerGUI

== 로봇 이벤트 로그 조회 ==

ManagerGUI -> MainService: RequestRobotEventLog (기간 필터 optional)
activate ManagerGUI
activate MainService
MainService -> Database: SearchRobotEventLog (필터)
activate Database
Database --> MainService: RobotEventLogResult (robot event list)
deactivate Database
MainService -> ManagerGUI: RobotEventLogResponse (event list)
deactivate MainService
deactivate ManagerGUI

== 작업 기록 조회 ==

ManagerGUI -> MainService: RequestWorkLog (기간 필터 optional)
activate ManagerGUI
activate MainService
MainService -> Database: SearchWorkLog (필터)
activate Database
Database --> MainService: WorkLogResult (work log list)
deactivate Database
MainService -> ManagerGUI: WorkLogResponse (work log list)
deactivate MainService
deactivate ManagerGUI

== 로봇 상태 조회 ==

ManagerGUI -> MainService: RequestRobotStatus (optional robot_id)
activate ManagerGUI
activate MainService
MainService -> Database: SearchRobotStatus (robot_id or 전체)
activate Database
Database --> MainService: RobotStatusResult (robot_id, battery, operation_state, carriage_state)
deactivate Database
MainService -> ManagerGUI: RobotStatusResponse (robot status)
deactivate MainService
deactivate ManagerGUI

== 로봇 등록 ==

ManagerGUI -> MainService: RequestRobotRegistration (robot_id, robot_info)
activate ManagerGUI
activate MainService
MainService -> Database: InsertRobot (robot info)
activate Database
Database --> MainService: InsertRobotResult (success/failure)
deactivate Database
MainService -> ManagerGUI: RobotRegistrationResponse (success/failure)
deactivate MainService
deactivate ManagerGUI

== 로봇 삭제 ==

ManagerGUI -> MainService: RequestRobotDeletion (robot_id)
activate ManagerGUI
activate MainService
MainService -> Database: DeleteRobot (robot_id)
activate Database
Database --> MainService: DeleteRobotResult (success/failure)
deactivate Database
MainService -> ManagerGUI: RobotDeletionResponse (success/failure)
deactivate MainService
deactivate ManagerGUI

@enduml
```

## Handle Received Data From Object Detector

```plantuml
@startuml
participant AIModule
participant ObjectDetector
participant MainService
participant MobileController
database Database

== AI 모듈이 추론 결과를 Object Detector에 전달 ==

AIModule -> ObjectDetector: AIResultReport (ai_task_id, inference_result)
activate AIModule
activate ObjectDetector
ObjectDetector -> ObjectDetector: Validate and prepare inference result
deactivate AIModule

== Object Detector가 Main Service에 결과 전달 ==

ObjectDetector -> MainService: InferenceResult (ai_task_id, object_type, confidence, location)
deactivate ObjectDetector
activate MainService

== Main Service가 결과 분석 및 대응 분기 ==

MainService -> MainService: AnalyzeInferenceResult (object_type)

alt 협업 로봇 탐지
    MainService -> MobileController: PauseCommand
    activate MobileController
else 사람 탐지
    MainService -> MobileController: EmergencyStopCommand
else 섹션 감지
    MainService -> MainService: UpdateTaskPhase
end

== 로봇이 명령 수신 확인 ==

MobileController -> MainService: CommandAck (result)
deactivate MobileController

== 선택적: 결과 및 조치 기록 ==

MainService -> Database: StoreInferenceResult (ai_task_id, result_data)
deactivate MainService

activate Database
deactivate Database

@enduml
```

## Handle Received Data From Mobile Controller

```plantuml
@startuml
participant MobileController
participant MainService
database Database

== 센서 데이터, 위치, 배터리, 상태 수신 ==

MobileController -> MainService: SensorDataReport (imu, lidar, ultrasonic, ir, fsr)
activate MobileController
activate MainService

MobileController -> MainService: LocationUpdate (position, timestamp)
MobileController -> MainService: BatteryStatusReport (battery_level, charging_status)
MobileController -> MainService: RobotStateUpdate (current_task_id, operation_state, carriage_state)
deactivate MobileController

MainService -> MainService: Update internal robot state

alt 배터리 부족
    MainService -> MobileController: MoveToChargingStation (charging_station_location)
    activate MobileController
    deactivate MobileController
end

== 상태 저장 ==

MainService -> Database: SaveRobotStatus (robot snapshot)
deactivate MainService
activate Database
deactivate Database

@enduml
```

## Received Request and Respond to Mobile Controller

```plantuml
@startuml
participant MainService
participant MobileController

== 작업 할당 ==

MainService -> MobileController: AssignTask (task_id, pickup_location, dropoff_location, item_id)
activate MainService
activate MobileController
MobileController -> MainService: TaskAck (task_id, result, reason)
alt Task Assignment Failed
    MainService -> MobileController: Retry or Abort Instruction
    deactivate MobileController
    deactivate MainService
end


== 이동 명령 ==

MainService -> MobileController: SendNavigationGoal (goal_location, task_id)
activate MainService
activate MobileController
MobileController -> MainService: NavigationAck (task_id, result, reason)
alt Navigation Failed
    MainService -> MainService: Trigger Local Replanning
    MainService -> MobileController: NewNavigationGoal (if replanned)
    deactivate MobileController
    deactivate MainService
end

== 작업 취소 ==

MainService -> MobileController: CancelCurrentTask (task_id)
activate MainService
activate MobileController
MobileController -> MainService: CancelTaskAck (task_id, result, reason)
deactivate MobileController
deactivate MainService

== 로봇이 다음 행동 요청 ==

MobileController -> MainService: RequestNextAction (robot_id, status)
activate MobileController
activate MainService
MainService -> MobileController: NextActionResponse (action)
deactivate MainService
deactivate MobileController

@enduml
```

# Mobile Controller

## Sensor data collection and state transfer flow

```plantuml
@startuml
participant SensorCollectorModule
participant MainService
participant StaffPC
participant ManagerPC

== 센서 데이터 수집 ==

activate SensorCollectorModule
SensorCollectorModule -> SensorCollectorModule: SensorDataReport (imu, lidar, ultrasonic, ir, fsr, pressure_sensor)

== 압력 센서 상태 처리 ==

SensorCollectorModule -> SensorCollectorModule: PressureSensorStatus (is_object_present)
deactivate SensorCollectorModule

== Main Service로 센서 상태 전송 ==

activate MainService
SensorCollectorModule -> MainService: SensorDataReport (robot_id, aggregated sensor data, pressure_sensor_status, timestamp)

== Main Service가 상태 갱신 ==

MainService -> MainService: UpdateSensorState (robot_id, sensor status including pressure sensor state)

== 압력 센서 상태를 StaffPC로 전송 (TCP) ==

MainService -> StaffPC: SendPressureSensorDataTCP (robot_id, pressure_sensor_status, timestamp)

== 압력 센서 상태를 ManagerPC로 전송 (ROS) ==

MainService -> ManagerPC: SendPressureSensorDataROS (robot_id, pressure_sensor_status, timestamp)
deactivate MainService
@enduml
```

## Battery flow

```plantuml
@startuml
participant MobileController
participant MainService
participant StaffPC
participant ManagerPC

== 배터리 상태 실시간 수집 및 전달 ==

activate MobileController
MobileController -> MainService: BatteryStatusReport (robot_id, battery_level, charging_status, timestamp)
deactivate MobileController

== Main Service가 상태 갱신 ==

activate MainService
MainService -> MainService: UpdateRobotHealthState (robot_id, battery_level, charging_status, timestamp)

== Staff PC로 배터리 상태 전달 (TCP 통신) ==

activate StaffPC
MainService -> StaffPC: SendBatteryStatusTCP (robot_id, battery_level, charging_status, timestamp)
deactivate StaffPC

== Manager PC로 배터리 상태 전달 (ROS 통신) ==

activate ManagerPC
MainService -> ManagerPC: SendBatteryStatusROS (robot_id, battery_level, charging_status, timestamp)
deactivate ManagerPC
deactivate MainService
@enduml
```

## Video Sender(Camera video capture and AI Server send flow)

```plantuml
@startuml
participant VideoSenderModule
participant AIServer_ObjectDetector
participant MainService

== 카메라에서 영상 프레임 캡처 ==

activate VideoSenderModule
VideoSenderModule -> VideoSenderModule: CapturedFrame (frame, timestamp)
VideoSenderModule -> VideoSenderModule: EncodeFrame (compression/formatting)

== AI Server로 프레임 전송 (robot_id 추가) ==

activate AIServer_ObjectDetector
VideoSenderModule -> AIServer_ObjectDetector: MediaFrame (encoded frame, metadata, robot_id)
AIServer_ObjectDetector -> AIServer_ObjectDetector: ReceiveFrame (handle incoming frame, robot_id)
deactivate VideoSenderModule

== ObjectDetector 결과를 MainService로 전송 ==

activate MainService
AIServer_ObjectDetector -> MainService: DetectionResult (robot_id, detected_objects, timestamp)
deactivate MainService

@enduml
```

## Command Reception and Motor/Actuator Control Flow

```plantuml
@startuml
participant MainService
participant MobileController_StateManager
participant MotorController
participant ManagerPC
participant StaffPC

== Main Service가 이동 명령 전송 ==

activate MainService
MainService -> MobileController_StateManager: MovementCommand (move/stop/rotate, target_location, shortest_path)
deactivate MainService

activate MobileController_StateManager
MobileController_StateManager -> MotorController: ExecuteMovementCommand (parsed command)
deactivate MobileController_StateManager

activate MotorController
MotorController -> MotorController: DriveMotor (speed, direction)

== 물체 감지 및 회피 명령 조건 체크 ==

loop 물체 감지 및 회피
    MotorController -> MobileController_StateManager: ObstacleDetected (distance_to_object)
    
    activate MobileController_StateManager
    MobileController_StateManager -> MainService: ObstacleAlert (distance_to_object, obstacle_position)
    deactivate MobileController_StateManager
    
    MotorController -> MotorController: (waiting for avoidance command)

    activate MainService
    MainService -> MobileController_StateManager: AvoidanceCommand (rotate_direction)
    deactivate MainService

    activate MobileController_StateManager
    MobileController_StateManager -> MotorController: ExecuteAvoidanceCommand (rotate_direction)
    deactivate MobileController_StateManager

    MotorController -> MotorController: DriveMotor (rotate to avoid obstacle)
end loop

== 이동 완료 or 실패 보고 ==

MotorController -> MobileController_StateManager: MovementFeedback (execution_result, current_position)

activate MobileController_StateManager
MobileController_StateManager -> MainService: CommandAck (success/failure)
deactivate MobileController_StateManager

alt 이동 실패
  activate MainService
  MainService -> ManagerPC: SendErrorReport (error_details, timestamp)
  MainService -> StaffPC: SendErrorReport (error_details, timestamp)
  
  MainService -> MobileController_StateManager: RetryMovementCommand (retry_command)
  deactivate MainService
end alt

== 정밀 정차 명령 처리 ==

activate MainService
MainService -> MobileController_StateManager: PreciseParkingCommand (target_location, tolerance ±5cm)
deactivate MainService

activate MobileController_StateManager
MobileController_StateManager -> MotorController: ExecutePreciseParking (target_location, tolerance ±5cm)
deactivate MobileController_StateManager

MotorController -> MotorController: FineAdjustPosition (within ±5cm)

MotorController -> MobileController_StateManager: ParkingFeedback (success/failure)

activate MobileController_StateManager
MobileController_StateManager -> MainService: ParkingAck (success/failure)
deactivate MobileController_StateManager

== 배터리 상태 모니터링 및 충전 명령 ==
loop 배터리 잔량 실시간 체크
    MotorController -> MobileController_StateManager: BatteryStatusReport (battery_level)

  activate MobileController_StateManager
  MobileController_StateManager -> MainService: BatteryStatusUpdate (battery_level)
  deactivate MobileController_StateManager

  activate MainService
  MainService -> MobileController_StateManager: ChargingCommand (charging_station_location)
  deactivate MainService
end loop

alt 배터리 잔량 <= 20
  activate MobileController_StateManager
  MobileController_StateManager -> MotorController: MoveToChargingStation (charging_station_location)
  deactivate MobileController_StateManager

  MotorController -> MotorController: DriveMotor (move to charging station)

  MotorController -> MobileController_StateManager: ChargingArrivalFeedback (arrival_status)
end alt

activate MobileController_StateManager
MobileController_StateManager -> MainService: ChargingArrivalAck (success/failure)
deactivate MobileController_StateManager

== Task 취소 요청 처리 ==
alt Task 취소 요청 받음
  activate MainService
  MainService -> MobileController_StateManager: CancelTaskRequest (when cart_state == 0)
  deactivate MainService

  activate MobileController_StateManager
  MobileController_StateManager -> MotorController: SetStateToStandby()
  MobileController_StateManager -> MainService: CancelAck (standby mode entered)
  MotorController -> MotorController: DriveMotor (speed, direction)
  deactivate MobileController_StateManager
  deactivate MotorController
end alt

@enduml
```




# AI Server

```plantuml
@startuml
participant "Video Sender" as Video
participant "File System" as File
participant "Object Detecter" as Detector
participant "AI Module" as AI
participant "Main Service" as Main

Video -> File : UDP stream media data
activate File
File -> File : write data to file path
File -> Detector : send media file path
deactivate File

activate Detector
Detector -> Detector : detect object in media
Detector -> AI : dispatch detected objects\nto AI modules
deactivate Detector

activate AI
AI -> AI : run interface on data
AI -> Main : send interface
deactivate AI

activate Main
Main -> Main : save interface to database
deactivate Main
@enduml
```

## Object Detecter

```plantuml
@startuml
participant MobileController
participant ObjectDetector
participant AIModule
participant MainService
database FileSystem

== 로봇에서 미디어 프레임 수신 ==

activate MobileController
MobileController -> ObjectDetector: MediaFrame (image frame, metadata)
deactivate MobileController

== 미디어 저장 ==

activate ObjectDetector
activate FileSystem
ObjectDetector -> FileSystem: SaveFrame (save image file)
deactivate FileSystem

== AI 모듈에 분석 요청 ==


ObjectDetector -> AIModule: AnalyzeFrame (image file path)


== AI 모듈이 분석 결과 반환 ==

activate AIModule
AIModule -> ObjectDetector: AIResultReport (ai_task_id, inference_result)
deactivate AIModule

== 결과 검증 및 정리 ==

ObjectDetector -> ObjectDetector: Validate and prepare inference result

== Main Service에 결과 전달 ==

activate MainService
ObjectDetector -> MainService: InferenceResult (ai_task_id, object_type, confidence, location)
deactivate ObjectDetector
deactivate MainService


@enduml
```