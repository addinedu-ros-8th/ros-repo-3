from sqlalchemy.orm import Session
from db.roscars_models import RosCars, User, ShoesModel, Task, Delivery, Location
from db.roscars_log_models import (
    RosCarEventLog,
    TaskEventLog,
    DeliveryEventLog,
    PrecisionStopLog,
    RobotTrajectoryLog,
    SensorFusionRawLog,
)

# 로봇 이벤트 로그 + 상태 정보
def get_roscar_event_detail(session: Session):
    return (
        session.query(
            RosCarEventLog.event_id,
            RosCarEventLog.roscar_id,
            RosCars.roscar_name,
            RosCars.operational_status,
            RosCars.roscar_ip_v4,
            RosCarEventLog.event_type,
            RosCarEventLog.event_timestamp,
        )
        .join(RosCars, RosCarEventLog.roscar_id == RosCars.roscar_id)
        .all()
    )


# 작업 이벤트 로그 + 신발 모델 + 위치 정보
def get_task_event_history(session: Session):
    return (
        session.query(
            TaskEventLog.event_id,
            TaskEventLog.task_id,
            Task.task_status,
            ShoesModel.name.label("shoes_name"),
            Location.name.label("location_name"),
            TaskEventLog.previous_event,
            TaskEventLog.new_event,
            TaskEventLog.changed_at,
        )
        .join(Task, TaskEventLog.task_id == Task.task_id)
        .join(ShoesModel, Task.shoes_id == ShoesModel.shoes_model_id)
        .join(Location, Task.location_id == Location.location_id)
        .all()
    )


# 배송 요청 이벤트 + 요청자 이름 + 로봇 이름
def get_delivery_log(session: Session):
    return (
        session.query(
            DeliveryEventLog.event_id,
            Delivery.delivery_id,
            User.user_name,
            RosCars.roscar_name,
            DeliveryEventLog.new_event.label("event_type"),
            DeliveryEventLog.changed_at,
        )
        .join(Delivery, DeliveryEventLog.delivery_id == Delivery.delivery_id)
        .join(User, Delivery.user_id == User.user_id)
        .join(RosCars, Delivery.roscar_id == RosCars.roscar_id)
        .all()
    )


# 정밀 정지 결과 + 로봇 이름
def get_precision_stop_result(session: Session):
    return (
        session.query(
            PrecisionStopLog.log_id,
            PrecisionStopLog.roscar_id,
            RosCars.roscar_name,
            PrecisionStopLog.task_id,
            PrecisionStopLog.is_success,
            PrecisionStopLog.deviation_cm,
            PrecisionStopLog.timestamp,
        )
        .join(RosCars, PrecisionStopLog.roscar_id == RosCars.roscar_id)
        .all()
    )


# 로봇 주행 이력 + 이름
def get_roscar_trajectory(session: Session):
    return (
        session.query(
            RobotTrajectoryLog.trajectory_id,
            RobotTrajectoryLog.roscar_id,
            RosCars.roscar_name,
            RobotTrajectoryLog.task_id,
            RobotTrajectoryLog.timestamp,
            RobotTrajectoryLog.position_x,
            RobotTrajectoryLog.position_y,
            RobotTrajectoryLog.velocity,
            RobotTrajectoryLog.heading_angle,
        )
        .join(RosCars, RobotTrajectoryLog.roscar_id == RosCars.roscar_id)
        .all()
    )


# 학습용 센서 로그 + 로봇 이름
def get_sensor_for_training(session: Session):
    return (
        session.query(
            SensorFusionRawLog.sensor_log_id,
            RosCars.roscar_name,
            SensorFusionRawLog.timestamp,
            SensorFusionRawLog.lidar_raw,
            SensorFusionRawLog.imu_data,
            SensorFusionRawLog.ultrasonic_data,
            SensorFusionRawLog.camera_frame_id,
        )
        .join(RosCars, SensorFusionRawLog.roscar_id == RosCars.roscar_id)
        .all()
    )

# TODO: insert_default_users
# TODO: insert_sample_logs

