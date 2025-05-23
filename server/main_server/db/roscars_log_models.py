import enum
from sqlalchemy import (
    Column, Integer, String, Boolean, Float, Enum, TIMESTAMP, JSON
)
from sqlalchemy.ext.declarative import declarative_base

# Base class for all models
RoscarsLogBase = declarative_base()

# Enum definitions
class EventType(enum.Enum):
    WAIT = 'WAIT'
    PROGRESS_START = 'PROGRESS_START'
    COMPLET = 'COMPLET'
    CANCEL = 'CANCEL'
    FAILE = 'FAILE'

class RosCarEventType(enum.Enum):
    PICKUP_START = 'PICKUP_START'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    COLLISION_AVOID = 'COLLISION_AVOID'
    ERROR = 'ERROR'
    CHARGING_START = 'CHARGING_START'
    CHARGING_COMPLET = 'CHARGING_COMPLET'

class ControlSource(enum.Enum):
    PATH_PLANNER = 'PATH_PLANNER'
    OBSTACLE_AVOIDANCE = 'OBSTACLE_AVOIDANCE'
    EMERGENCY_HANDLER = 'EMERGENCY_HANDLER'
    MANUAL_OVERRIDE = 'MANUAL_OVERRIDE'

# Tables
class DeliveryEventLog(RoscarsLogBase):
    __tablename__ = 'DeliveryEventLog'

    event_id = Column(Integer, primary_key=True)
    delivery_id = Column(Integer)
    previous_event = Column(Enum(EventType, name="event_type_enum"))
    new_event = Column(Enum(EventType, name="event_type_enum"))
    changed_at = Column(TIMESTAMP)
    changed_by_user_id = Column(Integer)

class TaskEventLog(RoscarsLogBase):
    __tablename__ = 'TaskEventLog'

    event_id = Column(Integer, primary_key=True)
    task_id = Column(Integer)
    previous_event = Column(Enum(EventType, name="event_type_enum"))
    new_event = Column(Enum(EventType, name="event_type_enum"))
    changed_at = Column(TIMESTAMP)

class RosCarEventLog(RoscarsLogBase):
    __tablename__ = 'RosCarEventLog'

    event_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    task_id = Column(Integer)
    event_type = Column(Enum(RosCarEventType, name="roscar_event_type_enum"))
    event_timestamp = Column(TIMESTAMP)

class InventoryEventLog(RoscarsLogBase):
    __tablename__ = 'InventoryEventLog'

    event_id = Column(Integer, primary_key=True)
    actor_user_id = Column(Integer)
    item_id = Column(Integer)
    roscar_id = Column(Integer)
    quantity = Column(Integer)
    event_timestamp = Column(TIMESTAMP)

class FileSystemLog(RoscarsLogBase):
    __tablename__ = 'FileSystemLog'

    file_log_id = Column(Integer, primary_key=True)
    image_path = Column(String(255))  # 길이 지정
    updatetime = Column(TIMESTAMP)

class PrecisionStopLog(RoscarsLogBase):
    __tablename__ = 'PrecisionStopLog'

    log_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    task_id = Column(Integer)
    is_success = Column(Boolean)
    deviation_cm = Column(Float)
    timestamp = Column(TIMESTAMP)

class RobotTrajectoryLog(RoscarsLogBase):
    __tablename__ = 'RobotTrajectoryLog'

    trajectory_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    task_id = Column(Integer)
    timestamp = Column(TIMESTAMP)
    position_x = Column(Float)
    position_y = Column(Float)
    velocity = Column(Float)
    heading_angle = Column(Float)

class SensorFusionRawLog(RoscarsLogBase):
    __tablename__ = 'SensorFusionRawLog'

    sensor_log_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    timestamp = Column(TIMESTAMP)
    lidar_raw = Column(JSON)
    imu_data = Column(JSON)
    ultrasonic_data = Column(JSON)
    camera_frame_id = Column(String(255))

class ControlCommandLog(RoscarsLogBase):
    __tablename__ = 'ControlCommandLog'

    command_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    timestamp = Column(TIMESTAMP)
    linear_velocity = Column(Float)
    angular_velocity = Column(Float)
    control_source = Column(Enum(ControlSource, name="control_source_enum"))

class TrainingSample(RoscarsLogBase):
    __tablename__ = 'TrainingSample'

    sample_id = Column(Integer, primary_key=True)
    sensor_log_id = Column(Integer)
    image_path = Column(String(255))  # 길이 지정
    label = Column(JSON)
    is_used = Column(Boolean)
    created_at = Column(TIMESTAMP)

class InferenceModel(RoscarsLogBase):
    __tablename__ = 'InferenceModel'

    model_id = Column(Integer, primary_key=True)
    name = Column(String(255))
    version = Column(String(255))
    description = Column(String(255))
    created_at = Column(TIMESTAMP)

# Note: Views can be queried via session.execute(text("SELECT ...")) or defined as read-only mapped classes with SQLAlchemy 2.0 if needed
