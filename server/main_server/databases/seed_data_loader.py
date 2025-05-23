import bcrypt
from datetime import datetime
from server.main_server.databases.models.roscars_models import (
    User, UserRole,
    ShoesModel, ColorName,
    RackLocation, ShoesInventory,
    QRCode, RosCars
)

class SeedDataLoader:
    def __init__(self, roscars_session):
        self.roscars_session = roscars_session

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
            RackLocation(name="R3-S1-A1", floor_level=3, zone_number=2, map_x=1.0, map_y=1.0, aruco_id=101,
                         timestamp=datetime.now()),
            RackLocation(name="R3-S2-B4", floor_level=3, zone_number=2, map_x=1.5, map_y=1.2, aruco_id=102,
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
            QRCode(qr_code_value="Nike Air Rift | 240 | BLACK | R4-G2-S2 | 3",
                   inventory=inventory_list[1]),
            QRCode(qr_code_value="Adidas Gazelle | 260 | HOTPINK | R5-G6-S1 | 7",
                   inventory=inventory_list[2]),  # ← 누락분 추가
            QRCode(qr_code_value="PUMA PALERMO TRAINERS | 240 | HYPERLINK BLUE-FLAME FLICKER-GUM | R7-G2-S2 | 5",
                   inventory=inventory_list[0]),  # ← 추가 QR
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

    def load_all(self):
        try:
            self.load_users()
            inventories = self.load_shoes_and_inventory()
            self.load_qrcodes(inventories)
            self.load_roscars()  # ← RosCars 로딩 추가
            self.roscars_session.commit()
            print("[Seed] 전체 커밋 완료")
        except Exception as e:
            self.roscars_session.rollback()
            print(f"[Seed] 오류 발생 → 롤백: {e}")
        finally:
            self.roscars_session.close()
