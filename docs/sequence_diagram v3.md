# 1. 사용자 단말 초기화 및 인증 (user_authentication)

```plantuml
' 로그인 및 초기 연결
' 포함: Login Flow, Staff PC 실행, Manager GUI 로그인

@startuml
title 사용자 단말 초기화 및 인증 흐름

actor User
participant "Staff GUI" as Staff
participant "Manager GUI" as Manager
participant "Main Service" as Main
database "User table" as DB

== Staff GUI 로그인 흐름 ==

User -> Staff : 프로그램 실행
activate Staff
Staff -> Main : 서버 연결 요청
activate Main
Main --> Staff : 서버 연결 결과\n{ success: true }
deactivate Main

alt 서버 연결 성공 시
    Staff -> Staff : 로그인 화면 표시

    User -> Staff : ID, Password 입력
    Staff -> Main : LoginRequest\n{ id: "staff001", password: "pw1234" }
    activate Main

    Main -> DB : SELECT * FROM users\nWHERE id = "staff001"
    activate DB
    DB --> Main : { id, name, role: "staff", passwordHash, permissions }
    deactivate DB

    Main -> Main : 입력 PW 해시와 DB 비밀번호 해시 비교

    alt 비밀번호 일치
        Main -> DB : SELECT * FROM gui_layouts\nWHERE user_id = "staff001"
        activate DB
        DB --> Main : { layoutData }
        deactivate DB

        Main --> Staff : LoginResponse\n{ success: true, layoutData }
    else 비밀번호 불일치
        Main --> Staff : LoginResponse\n{ success: false, reason: "Invalid credentials" }
    end
    deactivate Main
else 서버 연결 실패
    Staff -> Staff : 오류 팝업 표시
end
deactivate Staff


== Manager GUI 로그인 흐름 ==

User -> Manager : 프로그램 실행
activate Manager
Manager -> Main : 서버 연결 요청
activate Main
Main --> Manager : 서버 연결 결과\n{ success: true }
deactivate Main

alt 서버 연결 성공 시
    Manager -> Manager : 로그인 화면 표시

    User -> Manager : ID, Password 입력
    Manager -> Main : ManagerLoginRequest\n{ id: "admin01", password: "adminpass" }
    activate Main

    Main -> DB : SELECT * FROM users\nWHERE id = "admin01"
    activate DB
    DB --> Main : { id, name, role: "manager", passwordHash, permissions }
    deactivate DB

    Main -> Main : 입력 PW 해시와 DB 비밀번호 해시 비교

    alt 비밀번호 일치
        Main -> DB : SELECT * FROM admin_dashboard_data\nWHERE manager_id = "admin01"
        activate DB
        DB --> Main : dashboardData\n(statusSummary, robotHealth, taskStats, inventoryOverview)
        deactivate DB

        Main --> Manager : ManagerLoginResponse\n{ success: true, dashboardData }
    else 비밀번호 불일치
        Main --> Manager : ManagerLoginResponse\n{ success: false, reason: "Invalid credentials" }
    end
    deactivate Main
else 서버 연결 실패
    Manager -> Manager : 오류 팝업 표시
end
deactivate Manager

@enduml
```

---

# 2. 재고 조회 및 요청 (inventory_request_flow)

```plantuml
@startuml
title 재고 조회 및 요청 흐름

actor User
participant "Staff GUI" as Staff
participant "Main Service" as Main
participant "Mobile Controller" as Mobile
database "Item table" as DB

== 상품 정보 조회 ==

User -> Staff : QR 스캔
activate Staff
Staff -> Staff : QR 코드 파싱 → 상품 ID 추출\nitem_id=ITEM-1234

Staff -> Main : SearchItemRequest\nitem_id=ITEM-1234
activate Main
Main -> DB : SELECT * FROM items WHERE item_id = ITEM-1234
activate DB
DB --> Main : model=Nike Air, size=270, color=Black,\nquantity=5, location=R3-S2-B1
deactivate DB
Main --> Staff : SearchItemResponse\n모델명, 사이즈, 수량, 위치
deactivate Main

Staff -> Staff : GUI에 상품 정보 표시

== 재고 요청 ==

User -> Staff : [요청] 버튼 클릭
Staff -> Main : InventoryRequest\nitem_id=ITEM-1234, quantity=1, location=R3-S2-B1
activate Main

Main -> Mobile : PickupCommand\ntask_id=TASK-789, item_id=ITEM-1234,\nfrom=R3-S2-B1, to=PICKUP_ZONE
activate Mobile

Mobile -> Mobile : 경로 계산 및 이동
Mobile -> Mobile : 제품 집재
Mobile -> Main : ArrivalReport\ntask_id=TASK-789, status=arrived, location=PICKUP_ZONE
deactivate Mobile

Main -> Staff : TaskStatusUpdate\ntask_id=TASK-789, status=배송중
deactivate Main

Staff -> Staff : GUI에 배송중 상태 표시

== 사용자의 수령 확인 및 담기 ==

User -> Staff : 픽업 완료 후 [담기] 클릭
Staff -> Staff : 담기 처리 (캐시 등록, UI 갱신)
deactivate Staff

@enduml
```

---

# 3. 작업 생성 및 로봇 할당 (task_assignment_flow)

```plantuml
@startuml
title 작업 생성 및 로봇 할당 흐름

actor User
participant "Staff GUI" as Staff
participant "Main Service" as Main
participant "Mobile Controller" as Mobile
database "Task table" as DB

== 작업 요청 및 Task 생성 ==

User -> Staff : 상품 선택 후 작업 요청 클릭
activate Staff

Staff -> Main : CreateTaskRequest\nitem_id=ITEM-1234,\nfrom=R3-S2-B1, to=PACK_ZONE, priority=normal
activate Main

Main -> Main : Task 객체 생성\n(task_id=TASK-101, status=PENDING, timestamp=2025-04-30 14:23)
Main -> DB : INSERT INTO tasks (...) VALUES (...)
activate DB
DB --> Main : OK
deactivate DB

== 대기 로봇 검색 및 할당 ==

Main -> DB : SELECT * FROM robots WHERE status=IDLE
activate DB
DB --> Main : robot_list = [RB01, RB02]
deactivate DB

Main -> Main : 최적 로봇 선택 (예: 거리 기반)\nbest_robot=RB01

Main -> Mobile : AssignTask\ntask_id=TASK-101, item_id=ITEM-1234,\nfrom=R3-S2-B1, to=PACK_ZONE, priority=normal
activate Mobile

Mobile -> Main : TaskAck\ntask_id=TASK-101, result=success
alt Task 수락 성공
    Main -> Main : Update task status → IN_PROGRESS
    Main -> DB : UPDATE tasks SET status='IN_PROGRESS' WHERE task_id=TASK-101
    activate DB
    DB --> Main : OK
    deactivate DB
else Task 수락 거절
    Main -> Main : 다른 로봇 재할당 시도 또는 대기열 등록
end
deactivate Mobile

== 작업 진행 상황 보고 ==

loop 주기적 업데이트
    Mobile -> Main : TaskProgressUpdate\ntask_id=TASK-101, location=R2-S2, progress=75%
    activate Main
    Main -> DB : UPDATE tasks SET progress=75%, location='R2-S2'
    activate DB
    DB --> Main : OK
    deactivate DB
    deactivate Main
end

== 작업 완료 처리 ==

Mobile -> Main : TaskCompleteReport\ntask_id=TASK-101, result=SUCCESS
activate Main
Main -> DB : UPDATE tasks SET status='COMPLETED', completed_at=...
activate DB
DB --> Main : OK
deactivate DB

Main -> Staff : CreateTaskResponse\ntask_id=TASK-101, result=SUCCESS
deactivate Main
deactivate Staff

@enduml
```

---

# 4. 로봇 상태 모니터링 및 관리 (robot_monitoring_and_management)

```plantuml
@startuml
title 로봇 상태 모니터링 및 관리

actor Manager
participant "Manager GUI" as GUI
participant "Main Service" as Main
participant "Mobile Controller" as Mobile
database "Robot table" as DB

== 로봇 상태 확인 ==

Manager -> GUI : 상태 확인 메뉴 클릭
activate GUI
GUI -> Main : RequestRobotStatus\nrobot_id=RB-01
activate Main
Main -> Mobile : GetStatus\nrobot_id=RB-01
activate Mobile
Mobile --> Main : RobotStatus\nbattery=76%, status=DRIVING, carriage=LOADED
deactivate Mobile
Main --> GUI : RobotStatusResponse\nbattery=76%, status=DRIVING, carriage=LOADED
deactivate Main
deactivate GUI

== 로봇 등록 ==

Manager -> GUI : 등록 메뉴 → 정보 입력
activate GUI
GUI -> Main : RegisterRobotRequest\nrobot_id=RB-99, model=K2X, location=Dock-1
activate Main
Main -> DB : INSERT INTO robots (...) VALUES (...)
activate DB
DB --> Main : OK
deactivate DB
Main --> GUI : RegisterRobotResponse\nsuccess=true
deactivate Main
deactivate GUI

== 로봇 삭제 ==

Manager -> GUI : 로봇 선택 후 삭제 클릭
activate GUI
GUI -> Main : DeleteRobotRequest\nrobot_id=RB-99
activate Main
Main -> DB : DELETE FROM robots WHERE robot_id='RB-99'
activate DB
DB --> Main : OK
deactivate DB
Main --> GUI : DeleteRobotResponse\nsuccess=true
deactivate Main
deactivate GUI

== 로봇 위치 실시간 확인 ==

Manager -> GUI : 위치 보기 탭 선택
loop 주기적 위치 확인 (예: 5초 간격)
    GUI -> Main : RequestRobotLocation\nrobot_id=RB-01
    activate Main
    Main -> Mobile : GetLocation\nrobot_id=RB-01
    activate Mobile
    Mobile --> Main : LocationInfo\nx=13.5, y=4.2, z=0
    deactivate Mobile
    Main --> GUI : LocationInfo\nx=13.5, y=4.2, z=0
    deactivate Main
end
deactivate GUI

@enduml
```

---

# 5. 로그 및 기록 조회 흐름 (log_and_audit_flow)

```plantuml
@startuml
title 로그 및 기록 조회 전체 흐름

actor Manager
participant "Manager GUI" as GUI
participant "Main Service" as Main
database "Log DB" as DB

== 1. 작업 로그 조회 ==

Manager -> GUI : 작업 로그 탭 선택
activate GUI
GUI -> Main : RequestLogs\nlog_type=task, date_from=2025-04-01, date_to=2025-04-30, keyword="cancel"
activate Main

Main -> DB : SELECT * FROM task_logs\nWHERE created_at BETWEEN '2025-04-01' AND '2025-04-30'\nAND status='CANCELLED'
activate DB
DB --> Main : 2건 반환
note right
task_id=TASK-1001, status=CANCELLED, start=2025-04-05 10:23, robot_id=RB02  
task_id=TASK-1010, status=CANCELLED, start=2025-04-12 14:02, robot_id=RB04
end note
deactivate DB

Main --> GUI : LogResponse\nlog_type=task, count=2
deactivate Main
GUI -> GUI : 작업 로그 테이블 표시
deactivate GUI

== 2. 재고 로그 조회 ==

Manager -> GUI : 재고 로그 탭 선택
activate GUI
GUI -> Main : RequestLogs\nlog_type=inventory, date_from=2025-04-01, date_to=2025-04-30
activate Main

Main -> DB : SELECT * FROM inventory_logs\nWHERE timestamp BETWEEN '2025-04-01' AND '2025-04-30'
activate DB
DB --> Main : 1건 반환
note right
log_id=INV-344, item_id=ITEM-1234, action=update, quantity=+3, location=R2-S3, timestamp=2025-04-15 09:31
end note
deactivate DB

Main --> GUI : LogResponse\nlog_type=inventory, count=1
deactivate Main
GUI -> GUI : 재고 로그 테이블 표시
deactivate GUI

== 3. 로봇 이벤트 로그 조회 ==

Manager -> GUI : 로봇 이벤트 탭 선택
activate GUI
GUI -> Main : RequestLogs\nlog_type=robot_event, date_from=2025-04-01, date_to=2025-04-30
activate Main

Main -> DB : SELECT * FROM robot_event_logs\nWHERE timestamp BETWEEN '2025-04-01' AND '2025-04-30'
activate DB
DB --> Main : 2건 반환
note right
event_id=EVT-2001, robot_id=RB01, type=EmergencyStop, timestamp=2025-04-03 17:12  
event_id=EVT-2002, robot_id=RB03, type=PathBlocked, timestamp=2025-04-07 11:45
end note
deactivate DB

Main --> GUI : LogResponse\nlog_type=robot_event, count=2
deactivate Main
GUI -> GUI : 로봇 이벤트 로그 테이블 표시
deactivate GUI

== 4. 출고 이력 로그 조회 ==

Manager -> GUI : 출고 이력 탭 선택
activate GUI
GUI -> Main : RequestLogs\nlog_type=shipment, date_from=2025-04-01, date_to=2025-04-30
activate Main

Main -> DB : SELECT * FROM shipment_logs\nWHERE shipped_at BETWEEN '2025-04-01' AND '2025-04-30'
activate DB
DB --> Main : 1건 반환
note right
shipment_id=SHP-901, item_id=ITEM-8888, qty=2, shipped_at=2025-04-18 13:44, staff_id=U123
end note
deactivate DB

Main --> GUI : LogResponse\nlog_type=shipment, count=1
deactivate Main
GUI -> GUI : 출고 로그 테이블 표시
deactivate GUI

@enduml
```

---

# 6. 로봇 센서 및 상태 보고 흐름 (sensor_and_state_reporting)

```plantuml
@startuml
title 센서 및 상태 보고 흐름

participant "Mobile Controller" as Mobile
participant "Main Service" as Main
participant "Staff PC" as StaffPC
participant "Manager PC" as ManagerPC
database "Robot DB" as DB

== 1. 센서 데이터 수집 및 전송 ==

activate Mobile
Mobile -> Main : SensorDataReport\nimu=ok, lidar=ok, ultrasonic=3.2m, ir=1.1m,\npressure=1 (object detected), timestamp=14:03:21
activate Main

Main -> Main : UpdateSensorSnapshot\n(robot_id=RB02, status=OK)

== 2. 배터리 상태 보고 ==

Mobile -> Main : BatteryStatusReport\nbattery=72%, charging=false, timestamp=14:03:22

Main -> Main : UpdateBatteryState\n(robot_id=RB02, battery=72%)

== 3. 로봇 동작 상태 보고 ==

Mobile -> Main : RobotStateUpdate\ntask_id=TASK-101,\nstatus=DRIVING,\ncarriage=LOADED
Main -> Main : UpdateRobotState

== 4. 상태 저장 ==

Main -> DB : INSERT INTO robot_status_log\n(robot_id=RB02, task_id=TASK-101, battery=72%,\nstatus=DRIVING, sensors=OK, time=14:03:22)
activate DB
DB --> Main : OK
deactivate DB

== 5. 사용자 단말 전송 ==

Main -> StaffPC : SendStatusTCP\nrobot_id=RB02, battery=72%, status=DRIVING, pressure=1
activate StaffPC
StaffPC -> StaffPC : 상태 표시 UI 업데이트
deactivate StaffPC

Main -> ManagerPC : SendStatusROS\nrobot_id=RB02, sensors=OK, location=(x=14.2, y=4.9)
activate ManagerPC
ManagerPC -> ManagerPC : ROS 상태 노드에 publish
deactivate ManagerPC

deactivate Main
deactivate Mobile

@enduml
```

---

# 7. AI 기반 인식 및 추론 흐름 (ai_perception_pipeline)

```plantuml
@startuml
title AI 기반 인식 및 추론 흐름

participant "Video Sender" as Video
participant "Object Detector" as Detector
participant "AI Module" as AI
participant "Main Service" as Main
database "Inference DB" as DB

== 1. 영상 프레임 전송 ==

activate Video
Video -> Detector : MediaFrame\nrobot_id=RB02, timestamp=14:05:02, frame_id=F123
deactivate Video

activate Detector
Detector -> AI : AnalyzeFrame\nframe_path=/media/RB02/F123.jpg,\nmeta=robot_id=RB02, frame_id=F123
activate AI

== 2. AI 추론 수행 ==

AI -> AI : run inference\n(model=yolov8, input=F123)
AI -> Detector : AIResultReport\nframe_id=F123, object=Human, confidence=0.91
deactivate AI

== 3. 인식 결과 처리 ==

Detector -> Detector : validate result\n(Human, 91%)
Detector -> Main : InferenceResult\nrobot_id=RB02, object=Human,\nconfidence=91%, location=(x=12.3, y=4.4)
deactivate Detector

activate Main

== 4. 판단 및 대응 로직 ==

Main -> Main : AnalyzeInferenceResult

alt 감지 대상이 사람일 경우
    Main -> Main : 판단 = 위험 상황
    Main -> MobileController : SendCommand\nEmergencyStop
else 감지 대상이 협업로봇일 경우
    Main -> MobileController : SendCommand\nPause
else 기타 물체일 경우
    Main -> Main : TaskPhase 업데이트
end

== 5. 추론 결과 저장 ==

Main -> DB : StoreInference\nrobot_id=RB02, object=Human,\nconfidence=91%, time=14:05:03
activate DB
DB --> Main : OK
deactivate DB

deactivate Main

@enduml
```

---

# 8. 로봇 제어 및 동작 흐름 (robot_control_and_motor_flow)

```plantuml
@startuml
title 로봇 제어 및 동작 흐름

participant "Main Service" as Main
participant "Mobile Controller" as Mobile
participant "Motor Controller" as Motor
participant "Staff PC" as Staff
participant "Manager PC" as Manager

== 1. 이동 명령 시작 ==

activate Main
Main -> Mobile : MovementCommand\nrobot_id=RB02, destination=(x=14.5, y=3.2), speed=normal
activate Mobile
Mobile -> Motor : ExecuteMovement\npath=[(12.0,3.0) → (13.0,3.1) → (14.5,3.2)]
activate Motor

== 2. 장애물 감지 및 회피 ==

loop 장애물 감지 루프
    Motor -> Mobile : ObstacleDetected\ndistance=0.8m, direction=front-right
    Mobile -> Main : ObstacleAlert\nrobot_id=RB02, distance=0.8m, direction=FRONT_RIGHT
    Main -> Mobile : AvoidanceCommand\nrotate=15°, direction=LEFT
    Mobile -> Motor : ExecuteAvoidance\nrotate=15° left
end

== 3. 이동 피드백 ==

Motor -> Mobile : MovementFeedback\nposition=(14.5,3.2), result=SUCCESS
Mobile -> Main : CommandAck\nrobot_id=RB02, result=SUCCESS

deactivate Motor
deactivate Mobile

== 4. 이동 실패 시 분기 ==

alt 이동 실패
    Main -> Manager : SendErrorReport\nrobot_id=RB02, error=PathBlocked, time=14:10:43
    Main -> Staff : SendErrorReport\nrobot_id=RB02, error=PathBlocked, time=14:10:43
end

deactivate Main

@enduml
```
