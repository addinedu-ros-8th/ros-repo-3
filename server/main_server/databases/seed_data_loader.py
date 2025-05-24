import bcrypt
from datetime import datetime
from server.main_server.databases.models.roscars_models import (
    User, UserRole,
    ShoesModel, ColorName,
    RackLocation, ShoesInventory,
    QRCode, RosCars
)
from server.main_server.databases.models.roscars_log_models import (
    RoscarSensorFusionRawLog,
    RoscarTrajectoryLog,
    RosCarEventLog,
    RosCarDrivingEventLog,
    ControlCommandLog,
    PrecisionStopLog,
    DeliveryEventLog,
    TaskEventLog,
    RackSensorLog,
    FileSystemLog,
    DefaultEventType,
    RosCarEventType,
    DrivingEvent,
    ControlSource
)
class SeedDataLoader:
    def __init__(self, roscars_session, log_session):
        self.roscars_session = roscars_session
        self.log_session = log_session

    def _hash_password(self, plain_pw: str) -> str:
        return bcrypt.hashpw(plain_pw.encode("utf-8"), bcrypt.gensalt()).decode("utf-8")

    def load_users(self):
        print("[Seed] 사용자 생성 중...")
        users = [
            User(user_name="stest", user_role=UserRole.STAFF, password=self._hash_password("stest123")),
            User(user_name="mtest", user_role=UserRole.MANAGER, password=self._hash_password("mtest123")),
        ]
        self.roscars_session.add_all(users)
        print("[Seed] 사용자 생성 완료")

    def load_shoes_and_inventory(self):
        print("[Seed] 신발 모델 및 위치/재고 생성 중...")

        shoes_list = [
            ShoesModel(name="Puma Palermo Trainers", size=240, color=ColorName.HYPERLINK_BLUE_FLAME_FLICKER_GUM),
            ShoesModel(name="Nike Air Rift", size=240, color=ColorName.BLACK),
            ShoesModel(name="Adidas Gazelle", size=260, color=ColorName.HOTPINK), 
        ]
        self.roscars_session.add_all(shoes_list)

        locations = [
            RackLocation(name="R3-G4-S1", floor_level=3, zone_number=2, map_x=1.0, map_y=1.0, aruco_id=101,
                         timestamp=datetime.now()),
            RackLocation(name="R3-G2-S2", floor_level=3, zone_number=2, map_x=1.5, map_y=1.2, aruco_id=102,
                         timestamp=datetime.now()),
        ]
        self.roscars_session.add_all(locations)

        inventory_list = [
            ShoesInventory(location=locations[0], shoes_model=shoes_list[0],
                           quantity=5, timestamp=datetime.now()),
            ShoesInventory(location=locations[1], shoes_model=shoes_list[1],
                           quantity=3, timestamp=datetime.now()),
            ShoesInventory(location=locations[0], shoes_model=shoes_list[2],
                           quantity=7, timestamp=datetime.now()),
        ]
        self.roscars_session.add_all(inventory_list)

        print("[Seed] 신발 모델 및 재고 생성 완료")
        return inventory_list

    def load_qrcodes(self, inventory_list):
        print("[Seed] QR 코드 생성 중...")

        qrcodes = [
            QRCode(qr_code_value="Puma Palermo Trainers | 240 | HYPERLINK_BLUE_FLAME_FLICKER_GUM | R3-G4-S1 | 5",
                   inventory=inventory_list[0]),
            QRCode(qr_code_value="Nike Air Rift | 240 | BLACK | R3-G2-S2 | 3",
                   inventory=inventory_list[1]),
            QRCode(qr_code_value="Adidas Gazelle | 260 | HOTPINK | R3-G2-S2 | 7",
                   inventory=inventory_list[2]),  # ← 누락분 추가
        ]
        self.roscars_session.add_all(qrcodes)
        print("[Seed] QR 코드 생성 완료")

    def load_roscars(self):
        print("[Seed] RosCars 로봇 데이터 생성 중...")

        roscars = [
            RosCars(roscar_namespace="pinky_0830", battery_percentage=100, operational_status="STANDBY",
                    roscar_ip_v4="192.168.0.101", cart_id="cart_0830", cart_ip_v4="192.168.0.201"),
            RosCars(roscar_namespace="pinky_07db", battery_percentage=100, operational_status="STANDBY",
                    roscar_ip_v4="192.168.0.102", cart_id="cart_07db", cart_ip_v4="192.168.0.202"),
        ]
        self.roscars_session.add_all(roscars)
        print("[Seed] RosCars 생성 완료")
        
    def load_test_logs(self):
        print("[Seed] 테스트 로그 데이터 생성 중...")

        now = datetime.now()

        logs = [
            RoscarSensorFusionRawLog(
                roscar_id=1,
                lidar_raw={"points": [1, 2, 3]},
                imu_data={"accel": [0.1, 0.2, 0.3]},
                ultrasonic_data={"left": 1.1, "right": 1.2},
                camera_frame_id="frame_001",
                timestamp=now
            ),
            RoscarTrajectoryLog(
                roscar_id=1,
                task_id=100,
                position_x=12.34,
                position_y=56.78,
                velocity=0.45,
                heading_angle=90.0,
                timestamp=now
            ),
            RosCarEventLog(
                roscar_id=1,
                task_id=100,
                event_type=RosCarEventType.TASK_START,
                timestamp=now,
                camera_angle=45
            ),
            RosCarDrivingEventLog(
                roscar_id=1,
                driving_event=DrivingEvent.GO_TO_STANDBY_ZONE,
                timestamp=now
            ),
            ControlCommandLog(
                roscar_id=1,
                linear_velocity=1.2,
                angular_velocity=0.3,
                control_source=ControlSource.OBSTACLE_AVOIDANCE,
                timestamp=now
            ),
            PrecisionStopLog(
                roscar_id=1,
                task_id=100,
                is_success=True,
                deviation_cm=1.5,
                timestamp=now
            ),
            DeliveryEventLog(
                delivery_id=200,
                previous_event=DefaultEventType.WAIT,
                new_event=DefaultEventType.PROGRESS_START,
                user_id=1,
                timestamp=now
            ),
            TaskEventLog(
                task_id=100,
                previous_event=DefaultEventType.WAIT,
                current_event=DefaultEventType.PROGRESS_START,
                changed_at=now
            ),
            RackSensorLog(
                roscar_id=1,
                rack_id=10,
                rack_status="OCCUPIED",
                rack_position_x=5.0,
                rack_position_y=3.0,
                rack_position_z=1.8,
                timestamp=now
            ),
            FileSystemLog(
                roscar_id=1,
                file_path="/var/log/roscar/0830/sensor_data_20250524.json",
                timestamp=now
            )
        ]

        self.log_session.add_all(logs)
        print("[Seed] 테스트 로그 데이터 생성 완료")

    def load_all(self):
        try:
            self.load_users()
            inventories = self.load_shoes_and_inventory()
            self.load_qrcodes(inventories)
            self.load_roscars()
            self.load_test_logs()
            self.roscars_session.commit()
            self.log_session.commit()
            print("[Seed] 전체 커밋 완료")
        except Exception as e:
            self.roscars_session.rollback()
            self.log_session.rollback()
            print(f"[Seed] 오류 발생 → 롤백: {e}")
        finally:
            self.log_session.close()
            self.roscars_session.close()
