import bcrypt
from datetime import datetime
from databases.models.roscars_models import (
    User, UserRole,
    ShoesModel, ColorName,
    RackLocation, ShoesInventory,
    QRCode
)

class SeedDataLoader:
    def __init__(self, session):
        self.session = session

    def _hash_password(self, plain_pw: str) -> str:
        return bcrypt.hashpw(plain_pw.encode("utf-8"), bcrypt.gensalt()).decode("utf-8")

    def load_users(self):
        print("[Seed] 사용자 생성 중...")
        users = [
            User(user_name="stest", user_role=UserRole.STAFF, password=self._hash_password("stest123")),
            User(user_name="mtest", user_role=UserRole.MANAGER, password=self._hash_password("mtest123")),
        ]
        self.session.add_all(users)
        print("[Seed] 사용자 생성 완료")

    def load_shoes_and_inventory(self):
        print("[Seed] 신발 모델 및 위치/재고 생성 중...")

        shoes_list = [
            ShoesModel(name="Puma Palermo Trainers", size=240, color=ColorName.HYPERLINK_BLUE_FLAME_FLICKER_GUM),
            ShoesModel(name="Nike Air Rift", size=240, color=ColorName.BLACK),
            ShoesModel(name="Adidas Gazelle", size=260, color=ColorName.HOTPINK),
        ]
        self.session.add_all(shoes_list)

        locations = [
            RackLocation(name="R3-S1-A1", floor_level=3, zone_number=2, map_x=1.0, map_y=1.0, aruco_id=101,
                         updated_at=datetime.now(), destination="PACKING_ZONE"),
            RackLocation(name="R3-S2-B4", floor_level=3, zone_number=2, map_x=1.5, map_y=1.2, aruco_id=102,
                         updated_at=datetime.now(), destination="PACKING_ZONE"),
        ]
        self.session.add_all(locations)

        inventory_list = [
            ShoesInventory(location_id=locations[0].location_id, shoes_model_id=shoes_list[0].shoes_model_id,
                           quantity=5, last_updated=datetime.now()),
            ShoesInventory(location_id=locations[1].location_id, shoes_model_id=shoes_list[1].shoes_model_id,
                           quantity=3, last_updated=datetime.now()),
        ]
        self.session.add_all(inventory_list)

        print("[Seed] 신발 모델 및 재고 생성 완료")
        return inventory_list

    def load_qrcodes(self, inventory_list):
        print("[Seed] QR 코드 생성 중...")
        qrcodes = [
            QRCode(qr_code_value="QR20250430X001", inventory_id=inventory_list[0].inventory_id),
            QRCode(qr_code_value="QR20250430X002", inventory_id=inventory_list[1].inventory_id),
        ]
        self.session.add_all(qrcodes)
        print("[Seed] QR 코드 생성 완료")

    def load_all(self):
        try:
            self.load_users()
            inventories = self.load_shoes_and_inventory()
            self.load_qrcodes(inventories)
            self.session.commit()
            print("[Seed] 전체 커밋 완료")
        except Exception as e:
            self.session.rollback()
            print(f"[Seed] 오류 발생 → 롤백: {e}")
        finally:
            self.session.close()
