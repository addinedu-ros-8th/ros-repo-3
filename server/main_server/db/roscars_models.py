import enum
from sqlalchemy import (
    Column, Integer, String, Boolean, Float, Enum, ForeignKey, TIMESTAMP
)
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship

Base = declarative_base()

# Enum definitions
class UserRole(enum.Enum):
    ADMIN = 'ADMIN'
    WORKER = 'WORKER'

class ColorName(enum.Enum):
    BLACK = 'black'
    WHITE = 'white'
    SNOW = 'snow'
    RED = 'red'
    TOMATO = 'tomato'
    SALMON = 'salmon'
    CORAL = 'coral'
    PINK = 'pink'
    MAGENTA = 'magenta'
    HOTPINK = 'hotpink'
    RASPBERRY = 'raspberry'
    ORANGE = 'orange'
    CHOCOLATE = 'chocolate'
    YELLOW = 'yellow'
    KHAKI = 'khaki'
    GOLD = 'gold'
    GREEN = 'green'
    GREENYELLOW = 'greenyellow'
    OLIVE = 'olive'
    OLIVEDRAB = 'olivedrab'
    AZURE = 'azure'
    SKYBLUE = 'skyblue'
    AQUA = 'aqua'
    AQUAMARINE = 'aquamarine'
    CYAN = 'cyan'
    BLUE = 'blue'
    NAVY = 'navy'
    VIOLET = 'violet'
    LAVENDER = 'lavender'
    INDIGO = 'indigo'
    PURPLE = 'purple'

class OperationalStatus(enum.Enum):
    STANDBY = 'STANDBY'
    DRIVING = 'DRIVING'
    CHARGING = 'CHARGING'
    ERROR = 'ERROR'
    EMERGENCY_STOP = 'EMERGENCY_STOP'

class DrivingStatus(enum.Enum):
    PICKUP = 'PICKUP'
    DELIVERY = 'DELIVERY'
    RETURN = 'RETURN'
    PRECISION_STOP = 'PRECISION_STOP'
    GO_TO_STANDBY_ZONE = 'GO_TO_STANDBY_ZONE'
    GO_TO_CHARGING_ZONE = 'GO_TO_CHARGING_ZONE'
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
class User(Base):
    __tablename__ = 'User'

    user_id = Column(Integer, primary_key=True)
    user_name = Column(String(255))
    user_role = Column(Enum(UserRole))
    can_call_roscar = Column(Boolean)
    password = Column(String(255))

class ShoesModel(Base):
    __tablename__ = 'ShoesModel'

    shoes_model_id = Column(Integer, primary_key=True)
    name = Column(String(255))
    size = Column(Integer)
    color_name = Column(Enum(ColorName))

class Location(Base):
    __tablename__ = 'Location'

    location_id = Column(Integer, primary_key=True)
    name = Column(String(255))
    floor_level = Column(Integer)
    zone_number = Column(Integer)
    map_x = Column(Float)
    map_y = Column(Float)
    aruco_id = Column(Integer)
    updated_at = Column(TIMESTAMP)
    destination = Column(String(255))

class Inventory(Base):
    __tablename__ = 'Inventory'

    inventory_id = Column(Integer, primary_key=True)
    location_id = Column(Integer, ForeignKey('Location.location_id'))
    shoes_model_id = Column(Integer, ForeignKey('ShoesModel.shoes_model_id'))
    quantity = Column(Integer)
    last_updated = Column(TIMESTAMP)

    location = relationship("Location")
    shoes_model = relationship("ShoesModel")

class QRCode(Base):
    __tablename__ = 'QRCode'

    qrcode_id = Column(Integer, primary_key=True)
    inventory_id = Column(Integer, ForeignKey('Inventory.inventory_id'))
    qrcode_data = Column(String(255))

    inventory = relationship("Inventory")

class RosCars(Base):
    __tablename__ = 'RosCars'

    roscar_id = Column(Integer, primary_key=True)
    roscar_name = Column(String(255))
    battery_percentage = Column(Integer)
    operational_status = Column(Enum(OperationalStatus))
    roscar_ip_v4 = Column(String(15))

class RosCarDrivingStatus(Base):
    __tablename__ = 'RosCarDrivingStatus'

    driving_status_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer, ForeignKey('RosCars.roscar_id'))
    status_type = Column(Enum(DrivingStatus))
    is_enabled = Column(Boolean)

    roscar = relationship("RosCars")

class Delivery(Base):
    __tablename__ = 'Delivery'

    delivery_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer, ForeignKey('RosCars.roscar_id'))
    user_id = Column(Integer, ForeignKey('User.user_id'))
    driving_status_id = Column(Integer, ForeignKey('RosCarDrivingStatus.driving_status_id'))
    delivery_status = Column(Enum(DeliveryStatus))
    delivery_start_time = Column(TIMESTAMP)
    delivery_end_time = Column(TIMESTAMP)

    roscar = relationship("RosCars")
    user = relationship("User")
    driving_status = relationship("RosCarDrivingStatus")

class Task(Base):
    __tablename__ = 'Task'

    task_id = Column(Integer, primary_key=True)
    delivery_id = Column(Integer, ForeignKey('Delivery.delivery_id'))
    shoes_id = Column(Integer, ForeignKey('ShoesModel.shoes_model_id'))
    task_status = Column(Enum(TaskStatus))
    task_start_time = Column(TIMESTAMP)
    task_end_time = Column(TIMESTAMP)
    location_id = Column(Integer, ForeignKey('Location.location_id'))

    delivery = relationship("Delivery")
    shoes_model = relationship("ShoesModel")
    location = relationship("Location")

class PendingRosCars(Base):
    __tablename__ = 'PendingRosCars'

    pending_id = Column(Integer, primary_key=True)
    roscar_name = Column(String(255))
    roscar_ip_v4 = Column(String(15))
    requested_at = Column(TIMESTAMP)

class RosCarConnectionStatus(Base):
    __tablename__ = 'RosCarConnectionStatus'

    status_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer, ForeignKey('RosCars.roscar_id'))
    is_connected = Column(Boolean)
    last_heartbeat = Column(TIMESTAMP)

    roscar = relationship("RosCars")