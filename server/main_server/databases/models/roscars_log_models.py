import enum
from sqlalchemy import (
    Column, Integer, String, Boolean, Float, Enum, TIMESTAMP, JSON, func
)
from sqlalchemy.ext.declarative import declarative_base

# Base class for all models
RoscarsLogBase = declarative_base()

# Enum definitions
class DefaultEventType(enum.Enum):
    WAIT = 'WAIT'
    PROGRESS_START = 'PROGRESS_START'
    COMPLET = 'COMPLET'
    CANCEL = 'CANCEL'
    FAILE = 'FAILE'

class RosCarEventType(enum.Enum):
    TASK_START = 'TASK_START'
    TASK_DONE = 'TASK_DONE'
    OBJECT_DETECTED = 'OBJECT_DETECTED'
    PATH_MODIFIED = 'PATH_MODIFIED'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    COLLISION = 'COLLISION'
    CHARGING_START = 'CHARGING_START'
    CHARGING_DONE = 'CHARGING_DONE'

class DrivingEvent(enum.Enum):
    GO_TO_STANDBY_ZONE = 'GO_TO_STANDBY_ZONE'
    GO_TO_CHARGING_ZONE = 'GO_TO_CHARGING_ZONE'
    DISCONNECT = 'DISCONNECT'

class ControlSource(enum.Enum):
    OBSTACLE_AVOIDANCE = 'OBSTACLE_AVOIDANCE'
    EMERGENCY_HANDLER = 'EMERGENCY_HANDLER'
    PATH_PLANNER = 'PATH_PLANNER'

# Tables
class RoscarSensorFusionRawLog(RoscarsLogBase):
    __tablename__ = 'SensorFusionRawLog'

    sensor_log_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    lidar_raw = Column(JSON)
    imu_data = Column(JSON)
    ultrasonic_data = Column(JSON)
    camera_frame_id = Column(String(255))
    timestamp = Column(TIMESTAMP, server_default=func.now())

class RoscarTrajectoryLog(RoscarsLogBase):
    __tablename__ = 'RoscarTrajectoryLog'

    trajectory_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    task_id = Column(Integer)
    position_x = Column(Float)
    position_y = Column(Float)
    velocity = Column(Float)
    heading_angle = Column(Float)
    timestamp = Column(TIMESTAMP, server_default=func.now())

class RosCarEventLog(RoscarsLogBase):
    __tablename__ = 'RosCarEventLog'

    event_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    task_id = Column(Integer)
    event_type = Column(Enum(RosCarEventType, name="roscar_event_type_enum"))
    timestamp = Column(TIMESTAMP, server_default=func.now())

class RosCarDrivingEventLog(RoscarsLogBase):
    __tablename__ = 'RosCarDrivingEventLog'

    event_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    driving_event = Column(Enum(DrivingEvent, name='driving_event_enum'))
    timestamp = Column(TIMESTAMP, server_default=func.now())

class ControlCommandLog(RoscarsLogBase):
    __tablename__ = 'ControlCommandLog'

    command_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    linear_velocity = Column(Float)
    angular_velocity = Column(Float)
    control_source = Column(Enum(ControlSource, name="control_source_enum"))
    timestamp = Column(TIMESTAMP, server_default=func.now())

class PrecisionStopLog(RoscarsLogBase):
    __tablename__ = 'PrecisionStopLog'

    log_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    task_id = Column(Integer)
    is_success = Column(Boolean)
    deviation_cm = Column(Float)
    timestamp = Column(TIMESTAMP, server_default=func.now())

class DeliveryEventLog(RoscarsLogBase):
    __tablename__ = 'DeliveryEventLog'

    event_id = Column(Integer, primary_key=True)
    delivery_id = Column(Integer)
    previous_event = Column(Enum(DefaultEventType, name="previous_event_type_enum"))
    new_event = Column(Enum(DefaultEventType, name="new_event_type_enum"))
    user_id = Column(Integer)
    timestamp = Column(TIMESTAMP, server_default=func.now())

class TaskEventLog(RoscarsLogBase):
    __tablename__ = 'TaskEventLog'

    event_id = Column(Integer, primary_key=True)
    task_id = Column(Integer)
    previous_event = Column(Enum(DefaultEventType, name="previous_event_type_enum"))
    current_event = Column(Enum(DefaultEventType, name="current_event_type_enum"))
    changed_at = Column(TIMESTAMP, server_default=func.now())

class RackSensorLog(RoscarsLogBase):
    __tablename__ = 'RackSensorLog'

    sensor_log_id = Column(Integer, primary_key=True)
    roscar_id = Column(Integer)
    rack_id = Column(Integer)
    rack_status = Column(String(255))
    rack_position_x = Column(Float)
    rack_position_y = Column(Float)
    rack_position_z = Column(Float)
    timestamp = Column(TIMESTAMP, server_default=func.now())
