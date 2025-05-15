import enum
import bcrypt
from sqlalchemy import (
    Column, Integer, String, Boolean, Float, Enum, ForeignKey, TIMESTAMP, func
)
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship

RoscarsBase = declarative_base()

# Enum definitions
class UserRole(enum.Enum):
    STAFF = 'Staff'
    MANAGER = 'Manager'

class ColorName(enum.Enum):
    BLACK = 'black'
    WHITE = 'white'
    SNOW = 'snow'
    SALMON = 'salmon'
    HOTPINK = 'hotpink'
    RASPBERRY = 'raspberry'
    ORANGE = 'orange'
    CHOCOLATE = 'chocolate'
    SKYBLUE = 'skyblue'
    CYAN = 'cyan'
    BLUE = 'blue'
    NAVY = 'navy'
    VIOLET = 'violet'
    INDIGO = 'indigo'
    HYPERLINK_BLUE_FLAME_FLICKER_GUM = 'Hyperlink Blue-Flame Flicker-Gum'

class OperationalStatus(enum.Enum):
    STANDBY = 'STANDBY'
    DRIVING = 'DRIVING'
    CHARGING = 'CHARGING'
    ERROR = 'ERROR'
    EMERGENCY_STOP = 'EMERGENCY_STOP'

class DrivingPhase(enum.Enum):
    PICKUP = 'PICKUP'
    DELIVERY = 'DELIVERY'
    RETURN = 'RETURN'
    PRECISION_STOP = 'PRECISION_STOP'
    DISCONNECT = 'DISCONNECT'

class DeliveryStatus(enum.Enum):
    TO_DO = 'TO_DO'
    IN_PROGRESS = 'IN_PROGRESS'
    COMPLETING = 'COMPLETING'

class TaskStatus(enum.Enum):
    TO_DO = 'TO_DO'
    IN_PROGRESS = 'IN_PROGRESS'
    DONE = 'DONE'

# Tables
class RosCars(RoscarsBase):
    __tablename__ = 'RosCars'

    roscar_id = Column(Integer, primary_key=True)
    roscar_name = Column(String(255), unique=True)
    battery_percentage = Column(Integer)
    operational_status = Column(Enum(OperationalStatus, name='operational_status_enum'))
    roscar_ip_v4 = Column(String(15))

    deliveries = relationship("Delivery", back_populates="roscar")
    driving_phases = relationship("RosCarDrivingPhase", back_populates="roscar")

class RosCarDrivingPhase(RoscarsBase):
    __tablename__ = 'RosCarDrivingPhase'

    driving_phase_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer, ForeignKey('RosCars.roscar_id'))
    status_type = Column(Enum(DrivingPhase, name='driving_phase_enum'))
    is_enabled = Column(Boolean)

    roscar = relationship("RosCars", back_populates="driving_phases")

class Delivery(RoscarsBase):
    __tablename__ = 'Delivery'

    delivery_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer, ForeignKey('RosCars.roscar_id'))
    user_id = Column(Integer, ForeignKey('User.user_id'))
    driving_phase_id = Column(Integer, ForeignKey('RosCarDrivingPhase.driving_phase_id'))
    delivery_status = Column(Enum(DeliveryStatus, name='delivery_status_enum'))
    pickup_time = Column(TIMESTAMP)
    dropoff_time = Column(TIMESTAMP)

    user = relationship("User", back_populates="deliveries")
    roscar = relationship("RosCars", back_populates="deliveries")
    tasks = relationship("Task", back_populates="delivery")
    driving_phase = relationship("RosCarDrivingPhase")


class Task(RoscarsBase):
    __tablename__ = 'Task'

    task_id = Column(Integer, primary_key=True)
    delivery_id = Column(Integer, ForeignKey('Delivery.delivery_id'))
    shoes_model_id = Column(Integer, ForeignKey('ShoesModel.shoes_model_id'))
    status = Column(Enum(TaskStatus, name='task_status_enum'))
    start_time = Column(TIMESTAMP)
    end_time = Column(TIMESTAMP)
    location_id = Column(Integer, ForeignKey('RackLocation.location_id'))

    delivery = relationship("Delivery", back_populates="tasks")
    location = relationship("RackLocation")
    shoes_model = relationship("ShoesModel", back_populates="tasks")

class User(RoscarsBase):
    __tablename__ = 'User'

    user_id = Column(Integer, primary_key=True)
    user_name = Column(String(255))
    user_role = Column(Enum(UserRole, name='user_role_enum'))
    password = Column(String(60)) # bcrypt hash is 60 characters long

    deliveries = relationship("Delivery", back_populates="user")

    def check_password(self, input_pw: str) -> bool:
        """입력된 평문 비밀번호와 해시 비교"""
        if not self.password:
            return False
        return bcrypt.checkpw(input_pw.encode('utf-8'), self.password.encode('utf-8'))

    @staticmethod
    def is_password_valid(password: str) -> bool:
        """비밀번호 유효성 검사 (예: 최소 8자, 최대 64자)"""
        return 8 <= len(password) <= 64

class ShoesModel(RoscarsBase):
    __tablename__ = 'ShoesModel'

    shoes_model_id = Column(Integer, primary_key=True)
    name = Column(String(255))
    size = Column(Integer)
    color = Column(Enum(ColorName, name='color_enum'))

    inventories = relationship("ShoesInventory", back_populates="shoes_model")
    tasks = relationship("Task", back_populates="shoes_model")


class RackLocation(RoscarsBase):
    __tablename__ = 'RackLocation'

    location_id = Column(Integer, primary_key=True)
    name = Column(String(255))
    floor_level = Column(Integer)
    zone_number = Column(Integer)
    map_x = Column(Float)
    map_y = Column(Float)
    aruco_id = Column(Integer)
    destination = Column(String(255))
    timestamp = Column(TIMESTAMP, server_default=func.now())

    inventories = relationship("ShoesInventory", back_populates="location")


class ShoesInventory(RoscarsBase):
    __tablename__ = 'ShoesInventory'

    inventory_id = Column(Integer, primary_key=True)
    location_id = Column(Integer, ForeignKey('RackLocation.location_id'))
    shoes_model_id = Column(Integer, ForeignKey('ShoesModel.shoes_model_id'))
    quantity = Column(Integer)
    timestamp = Column(TIMESTAMP, server_default=func.now())

    location = relationship("RackLocation", back_populates="inventories")
    shoes_model = relationship("ShoesModel", back_populates="inventories")
    qrcodes = relationship("QRCode", back_populates="inventory")


class QRCode(RoscarsBase):
    __tablename__ = 'QRCode'

    qrcode_id = Column(Integer, primary_key=True)
    inventory_id = Column(Integer, ForeignKey('ShoesInventory.inventory_id'))
    qr_code_value = Column(String(255))

    inventory = relationship("ShoesInventory", back_populates="qrcodes")

