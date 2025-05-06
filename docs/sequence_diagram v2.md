# 1. 사용자 단말 초기화 및 인증 (user_authentication)

```plantuml
' 로그인 및 초기 연결
' 포함: Login Flow, Staff PC 실행, Manager GUI 로그인

@startuml
actor User
participant "Staff GUI" as Staff
participant "Manager GUI" as Manager
participant "Main Service" as Main
database Database

User -> Staff : 프로그램 실행
activate Staff
Staff -> Main : 서버 연결 시도
activate Main
Main --> Staff : 서버 연결 결과\n(result)
deactivate Main

Staff -> Main : 로그인 요청 (ID, password)
activate Main
Main -> Database : SearchUserByName
Database --> Main : 사용자 정보
Main -> Main : 비밀번호 검증
Main --> Staff : 로그인 결과 (성공/실패)
deactivate Main
deactivate Staff

User -> Manager : 프로그램 실행 및 로그인
activate Manager
Manager -> Main : 로그인 요청 (ID, password)
activate Main
Main -> Database : SearchUserByName
Database --> Main : 사용자 정보
Main -> Main : 비밀번호 검증
Main --> Manager : 로그인 결과 (성공/실패)
deactivate Main
deactivate Manager
@enduml
```

---

# 2. 재고 조회 및 요청 (inventory_request_flow)

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
Main -> Main : DB 검색 (상품 ID)
Main --> Staff : 재고 정보 응답
deactivate Main
Staff -> Staff : 상품 정보 표시

Staff -> Main : 재고 요청
activate Main
Main -> Mobile : 픽업 요청
activate Mobile
Mobile -> Mobile : 운송 수행
Mobile --> Main : 도착 알림
Main --> Staff : 배송중 상태 표시
deactivate Mobile
deactivate Main

Staff -> Staff : 픽업 완료 → 담기
@enduml
```

---

# 3. 작업 생성 및 로봇 할당 (task_assignment_flow)

```plantuml
@startuml
participant "Staff GUI" as Staff
participant "Main Service" as Main
participant "Mobile Controller" as Mobile
database Database

Staff -> Main : 작업 생성 요청 (item_id, 위치 등)
activate Main
Main -> Main : Task 객체 생성
Main -> Database : InsertTask

Main -> Database : QueryIdleRobots
Database --> Main : IDLE 로봇 목록
Main -> Main : 최적 로봇 선택
Main -> Mobile : AssignTask

Mobile -> Main : TaskAck (성공/거절)
alt 성공
  Main -> Main : 상태 업데이트
else 거절
  Main -> Main : 다른 로봇 시도 또는 대기열 등록
end

Mobile -> Main : 작업 진행 상황 보고
Main -> Database : 진행 상황 갱신

Mobile -> Main : 작업 완료 보고
Main -> Database : 완료 상태 저장
Main -> Staff : 작업 응답 (성공/실패)
@enduml
```

---

# 4. 로봇 상태 모니터링 및 관리 (robot_monitoring_and_management)

```plantuml
@startuml
actor Manager
participant "Manager GUI" as GUI
participant "Main Service" as Main
participant "Mobile Controller" as Mobile

Manager -> GUI : 로봇 상태 확인
GUI -> Main : 상태 요청
Main -> Mobile : 상태 질의
Mobile --> Main : 상태 정보 (battery, status 등)
Main --> GUI : 상태 응답

Manager -> GUI : 로봇 등록/삭제
GUI -> Main : 등록/삭제 요청
Main -> Main : DB 갱신
Main --> GUI : 결과 응답

Manager -> GUI : 로봇 위치 확인
loop 주기적 요청
  GUI -> Main : 위치 요청
  Main -> Mobile : 위치 요청
  Mobile --> Main : 위치 정보 전송
  Main --> GUI : 위치 응답
end
@enduml
```

---

# 5. 로그 및 기록 조회 흐름 (log_and_audit_flow)

```plantuml
@startuml
actor Manager
participant "Manager GUI" as GUI
participant "Main Service" as Main
database Database

Manager -> GUI : 로그/기록 조회
GUI -> Main : 로그 요청 (필터)
Main -> Database : 로그 조회
Database --> Main : 결과
Main --> GUI : 로그 응답 (출고, 재고, 작업, 이벤트)
@enduml
```

---

# 6. 로봇 센서 및 상태 보고 흐름 (sensor_and_state_reporting)

```plantuml
@startuml
participant MobileController
participant MainService
participant StaffPC
participant ManagerPC
database Database

MobileController -> MainService : SensorDataReport
MobileController -> MainService : BatteryStatusReport
MobileController -> MainService : RobotStateUpdate

MainService -> Database : 상태 저장
MainService -> StaffPC : 상태 전송 (TCP)
MainService -> ManagerPC : 상태 전송 (ROS)
@enduml
```

---

# 7. AI 기반 인식 및 추론 흐름 (ai_perception_pipeline)

```plantuml
@startuml
participant VideoSender
participant ObjectDetector
participant AIModule
participant MainService

VideoSender -> ObjectDetector : MediaFrame
ObjectDetector -> AIModule : AnalyzeFrame
AIModule -> ObjectDetector : AIResultReport
ObjectDetector -> MainService : InferenceResult

MainService -> MainService : 대응 결정
MainService -> Database : 결과 저장 (선택)
@enduml
```

---

# 8. 로봇 제어 및 동작 흐름 (robot_control_and_motor_flow)

```plantuml
@startuml
participant MainService
participant MobileController
participant MotorController
participant StaffPC
participant ManagerPC

MainService -> MobileController : MovementCommand
MobileController -> MotorController : ExecuteMovement

loop 장애물 감지
  MotorController -> MobileController : ObstacleDetected
  MobileController -> MainService : ObstacleAlert
  MainService -> MobileController : AvoidanceCommand
  MobileController -> MotorController : ExecuteAvoidance
end

MotorController -> MobileController : MovementFeedback
MobileController -> MainService : CommandAck

alt 실패 시
  MainService -> ManagerPC : 오류 보고
  MainService -> StaffPC : 오류 보고
end
@enduml
```
