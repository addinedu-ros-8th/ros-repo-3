from sqlalchemy.orm import Session
from server.main_server.databases.models.roscars_models import RosCars, User, ShoesModel, Task, Delivery, RackLocation
from server.main_server.databases.models.roscars_log_models import (
    RosCarEventLog,
    TaskEventLog,
    DeliveryEventLog,
    PrecisionStopLog,
    RosCarDrivingEventLog,
    RoscarTrajectoryLog,
    RoscarSensorFusionRawLog,
    ControlCommandLog,
    FileSystemLog,
    RackSensorLog,
)

class RoscarLogQuery:
    """roscar_log DB 로그 조회 전용"""
    def __init__(self, session: Session):
        self.session = session

    # 로봇 이벤트 로그 + 상태 정보
    def get_roscar_event_detail(self):
        return self.session.query(
                RosCarEventLog.event_id,
                RosCarEventLog.roscar_id,
                RosCars.roscar_name,
                RosCars.operational_status,
                RosCars.roscar_ip_v4,
                RosCarEventLog.event_type,
                RosCarEventLog.event_timestamp,
            ).join(RosCars, RosCarEventLog.roscar_id == RosCars.roscar_id).all()

    # 작업 이벤트 로그 + 신발 모델 + 위치 정보
    def get_task_event_history(self):
        return self.session.query(
            TaskEventLog.event_id,
            TaskEventLog.task_id,
            Task.status,
            ShoesModel.name.label("shoes_name"),
            RackLocation.name.label("location_name"),
            TaskEventLog.previous_event,
            TaskEventLog.current_event,
            TaskEventLog.changed_at,
        ).join(Task, TaskEventLog.task_id == Task.task_id)\
         .join(ShoesModel, Task.shoes_model_id == ShoesModel.shoes_model_id)\
         .join(RackLocation, Task.location_id == RackLocation.location_id).all()

    # 배송 요청 이벤트 + 요청자 이름 + 로봇 이름
    def get_delivery_log(self):
        return self.session.query(
            DeliveryEventLog.event_id,
            Delivery.delivery_id,
            User.user_name,
            RosCars.roscar_name,
            DeliveryEventLog.new_event.label("event_type"),
            DeliveryEventLog.timestamp,
        ).join(Delivery, DeliveryEventLog.delivery_id == Delivery.delivery_id)\
         .join(User, Delivery.user_id == User.user_id)\
         .join(RosCars, Delivery.roscar_id == RosCars.roscar_id).all()

    # 정밀 정지 결과 + 로봇 이름
    def get_precision_stop_result(self):
        return self.session.query(
            PrecisionStopLog.log_id,
            PrecisionStopLog.roscar_id,
            RosCars.roscar_name,
            PrecisionStopLog.task_id,
            PrecisionStopLog.is_success,
            PrecisionStopLog.deviation_cm,
            PrecisionStopLog.timestamp,
        ).join(RosCars, PrecisionStopLog.roscar_id == RosCars.roscar_id).all()

    # 로봇 주행 이력 + 이름
    def get_roscar_trajectory(self):
        return self.session.query(
            RoscarTrajectoryLog.trajectory_id,
            RoscarTrajectoryLog.roscar_id,
            RosCars.roscar_name,
            RoscarTrajectoryLog.task_id,
            RoscarTrajectoryLog.timestamp,
            RoscarTrajectoryLog.position_x,
            RoscarTrajectoryLog.position_y,
            RoscarTrajectoryLog.velocity,
            RoscarTrajectoryLog.heading_angle,
        ).join(RosCars, RoscarTrajectoryLog.roscar_id == RosCars.roscar_id).all()

    def get_roscar_driving_event_log(self):
        return self.session.query(
            RosCarDrivingEventLog.event_id,
            RosCarDrivingEventLog.roscar_id,
            RosCars.roscar_name,
            RosCarDrivingEventLog.driving_event,
            RosCarDrivingEventLog.timestamp,
        ).join(RosCars, RosCarDrivingEventLog.roscar_id == RosCars.roscar_id).all()

    # 학습용 센서 로그 + 로봇 이름
    def get_sensor_for_training(self):
        return self.session.query(
            RoscarSensorFusionRawLog.sensor_log_id,
            RosCars.roscar_name,
            RoscarSensorFusionRawLog.timestamp,
            RoscarSensorFusionRawLog.lidar_raw,
            RoscarSensorFusionRawLog.imu_data,
            RoscarSensorFusionRawLog.ultrasonic_data,
            RoscarSensorFusionRawLog.camera_frame_id,
        ).join(RosCars, RoscarSensorFusionRawLog.roscar_id == RosCars.roscar_id).all()

    def get_control_command_log(self):
        return self.session.query(
            ControlCommandLog.command_id,
            ControlCommandLog.roscar_id,
            ControlCommandLog.timestamp,
            ControlCommandLog.linear_velocity,
            ControlCommandLog.angular_velocity,
            ControlCommandLog.control_source,
        ).all()

    def get_filesystem_log(self):
        return self.session.query(
            FileSystemLog.log_id,
            FileSystemLog.file_path,
            FileSystemLog.timestamp,
        ).all()

    def get_rack_sensor_log(self):
        return self.session.query(
            RackSensorLog.sensor_log_id,
            RackSensorLog.roscar_id,
            RackSensorLog.rack_id,
            RackSensorLog.rack_status,
            RackSensorLog.rack_position_x,
            RackSensorLog.rack_position_y,
            RackSensorLog.rack_position_z,
            RackSensorLog.timestamp,
        ).all()

class RoscarQuery:
    """roscars 운영 DB 쿼리 전용"""
    def __init__(self, session: Session):
        self.session = session

    def is_user_name_taken(self, user_name: str) -> bool:
        return self.session.query(User).filter_by(user_name=user_name).first() is not None

    def get_available_roscars(self):
        return self.session.query(RosCars)\
            .filter(RosCars.operational_status == 'STANDBY').all()

    def get_user_by_name(self, user_name: str):
        return self.session.query(User).filter_by(user_name=user_name).first()

    def get_shoes_model_by_qrcode(self, qr_code_value: str):
        from databases.models.roscars_models import QRCode, ShoesInventory
        return self.session.query(
            ShoesModel.name,
            ShoesModel.size,
            ShoesModel.color,
            ShoesInventory.quantity,
            RackLocation.name.label("location"),
        ).join(ShoesInventory, ShoesModel.shoes_model_id == ShoesInventory.shoes_model_id)\
         .join(QRCode, QRCode.inventory_id == ShoesInventory.inventory_id)\
         .join(RackLocation, ShoesInventory.location_id == RackLocation.location_id)\
         .filter(QRCode.qr_code_value == qr_code_value).first()

    def get_task_event_log_by_task_id(self, task_id: int):
        return self.session.query(TaskEventLog)\
            .filter(TaskEventLog.task_id == task_id).order_by(TaskEventLog.changed_at).all()

    def get_delivery_event_log_by_delivery_id(self, delivery_id: int):
        return self.session.query(DeliveryEventLog)\
            .filter(DeliveryEventLog.delivery_id == delivery_id).order_by(DeliveryEventLog.timestamp).all()

    def get_task_by_id(self, task_id: int):
        return self.session.query(Task).filter_by(task_id=task_id).first()

    def get_delivery_by_id(self, delivery_id: int):
        return self.session.query(Delivery).filter_by(delivery_id=delivery_id).first()

    def get_roscar_by_id(self, roscar_id: int):
        return self.session.query(RosCars).filter_by(roscar_id=roscar_id).first()

    def get_all_locations(self):
        return self.session.query(RackLocation).all()

    def get_all_users(self):
        return self.session.query(User).all()
