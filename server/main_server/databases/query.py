from sqlalchemy.orm import Session
from server.main_server.databases.models.roscars_models import RosCars, User, ShoesModel, Task, Delivery, RackLocation, QRCode, ShoesInventory
from server.main_server.databases.models.roscars_log_models import (
    TaskEventLog, DeliveryEventLog, RosCarEventLog, PrecisionStopLog,
    RoscarTrajectoryLog, RosCarDrivingEventLog, RoscarSensorFusionRawLog,
    ControlCommandLog, FileSystemLog, RackSensorLog
)

class MainServiceQuery:
    def __init__(self, roscars_session: Session, roscars_log_session: Session):
        self.roscars_session = roscars_session
        self.roscars_log_session = roscars_log_session

    # 운영 DB 사용
    def get_user_by_name(self, user_name: str):
        return self.roscars_session.query(User).filter_by(user_name=user_name).first()

    def is_user_name_taken(self, user_name: str) -> bool:
        return self.get_user_by_name(user_name) is not None

    def get_available_roscars(self):
        return self.roscars_session.query(RosCars)\
            .filter(RosCars.operational_status == 'STANDBY').all()

    def get_task_by_id(self, task_id: int):
        return self.roscars_session.query(Task).filter_by(task_id=task_id).first()

    def get_delivery_by_id(self, delivery_id: int):
        return self.roscars_session.query(Delivery).filter_by(delivery_id=delivery_id).first()

    def get_roscar_by_id(self, roscar_id: int):
        return self.roscars_session.query(RosCars).filter_by(roscar_id=roscar_id).first()
    
    def get_roscar_id_by_name(self, roscar_name: str) -> int | None:
        """roscar_name → roscar_id"""
        roscar = self.roscars_session.query(RosCars)\
            .filter_by(roscar_name=roscar_name).first()
        return roscar.roscar_id if roscar else None

    def get_all_locations(self):
        return self.roscars_session.query(RackLocation).all()

    def get_all_users(self):
        return self.roscars_session.query(User).all()

    def get_shoes_model_by_qrcode(self, qr_code_value: str):
        return self.roscars_session.query(
            ShoesModel.name,
            ShoesModel.size,
            ShoesModel.color,
            ShoesInventory.quantity,
            RackLocation.name.label("location"),
        ).join(ShoesInventory, ShoesModel.shoes_model_id == ShoesInventory.shoes_model_id)\
         .join(QRCode, QRCode.inventory_id == ShoesInventory.inventory_id)\
         .join(RackLocation, ShoesInventory.location_id == RackLocation.location_id)\
         .filter(QRCode.qr_code_value == qr_code_value).first()
    
    def get_model_id_by_name(self, name: str) -> int | None:
        """ShoesModel.name → shoes_model_id"""
        m = self.roscars_session.query(ShoesModel)\
            .filter_by(name=name).first()
        return m.shoes_model_id if m else None
    def get_location_id_by_name(self, rack_name: str) -> int | None:
        """RackLocation.name → location_id"""
        loc = self.roscars_session.query(RackLocation)\
            .filter_by(name=rack_name).first()
        return loc.location_id if loc else None

    def get_task_event_log_by_task_id(self, task_id: int):
        logs = self.roscars_log_session.query(TaskEventLog)\
            .filter(TaskEventLog.task_id == task_id)\
            .order_by(TaskEventLog.changed_at).all()

        task = self.get_task_by_id(task_id)
        shoes_model = task.shoes_model if task else None
        location = task.location if task else None

        return [{
            "event_id": log.event_id,
            "task_id": log.task_id,
            "previous_event": log.previous_event.name,
            "current_event": log.current_event.name,
            "changed_at": log.changed_at,
            "shoes_name": shoes_model.name if shoes_model else None,
            "location_name": location.name if location else None
        } for log in logs]

    def get_delivery_event_log_by_delivery_id(self, delivery_id: int):
        logs = self.roscars_log_session.query(DeliveryEventLog)\
            .filter(DeliveryEventLog.delivery_id == delivery_id)\
            .order_by(DeliveryEventLog.timestamp).all()

        delivery = self.get_delivery_by_id(delivery_id)
        user = delivery.user if delivery else None
        roscar = delivery.roscar if delivery else None

        return [{
            "event_id": log.event_id,
            "delivery_id": log.delivery_id,
            "timestamp": log.timestamp,
            "event_type": log.new_event.name,
            "user_name": user.user_name if user else None,
            "roscar_name": roscar.roscar_name if roscar else None,
        } for log in logs]

    def get_all_delivery_logs(self):
        logs = self.roscars_log_session.query(DeliveryEventLog)\
            .order_by(DeliveryEventLog.timestamp).all()

        result = []
        for log in logs:
            delivery = self.get_delivery_by_id(log.delivery_id)
            user = delivery.user if delivery else None
            roscar = delivery.roscar if delivery else None

            result.append({
                "event_id": log.event_id,
                "delivery_id": log.delivery_id,
                "timestamp": log.timestamp,
                "event_type": log.new_event.name,
                "user_name": user.user_name if user else None,
                "roscar_name": roscar.roscar_name if roscar else None,
            })
        return result

    # 로봇 이벤트 로그 + 상태 정보
    def get_roscar_event_detail(self):
        # 로봇 이벤트 로그 조회 (roscars_log DB)
        logs = self.roscars_log_session.query(RosCarEventLog).all()

        # 운영 DB에서 roscar 정보 사전 조회 (roscars DB)
        roscars = self.roscars_session.query(RosCars).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "event_id": log.event_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map.get(log.roscar_id).roscar_name if log.roscar_id in roscar_map else None,
            "operational_status": roscar_map.get(log.roscar_id).operational_status if log.roscar_id in roscar_map else None,
            "roscar_ip_v4": roscar_map.get(log.roscar_id).roscar_ip_v4 if log.roscar_id in roscar_map else None,
            "cart_id": roscar_map.get(log.roscar_id).cart_id if log.roscar_id in roscar_map else None,
            "cart_ip_v4": roscar_map.get(log.roscar_id).cart_ip_v4 if log.roscar_id in roscar_map else None,
            "event_type": log.event_type.name,
            "event_timestamp": log.event_timestamp
        } for log in logs]

    # 작업 이벤트 로그 + 신발 모델 + 위치 정보
    def get_task_event_history(self):
        logs = self.roscars_log_session.query(TaskEventLog)\
            .order_by(TaskEventLog.changed_at).all()

        # 관련 Task ID 추출
        task_ids = [log.task_id for log in logs]

        # 운영 DB에서 Task, ShoesModel, RackLocation 사전 조회
        tasks = self.roscars_session.query(Task)\
            .filter(Task.task_id.in_(task_ids)).all()

        # 연관 관계 추적용 dict 구성
        task_map = {task.task_id: task for task in tasks}
        shoes_model_map = {
            task.task_id: task.shoes_model for task in tasks if task.shoes_model is not None
        }
        location_map = {
            task.task_id: task.location for task in tasks if task.location is not None
        }

        # 병합 후 결과 생성
        return [{
            "event_id": log.event_id,
            "task_id": log.task_id,
            "status": task_map[log.task_id].status if log.task_id in task_map else None,
            "shoes_name": shoes_model_map.get(log.task_id).name if log.task_id in shoes_model_map else None,
            "location_name": location_map.get(log.task_id).name if log.task_id in location_map else None,
            "previous_event": log.previous_event.name,
            "current_event": log.current_event.name,
            "changed_at": log.changed_at
        } for log in logs]

    # 정밀 정지 결과 + 로봇 이름
    def get_delivery_log(self):
        logs = self.roscars_log_session.query(DeliveryEventLog)\
            .order_by(DeliveryEventLog.timestamp).all()

        # 관련 Delivery ID 추출
        delivery_ids = [log.delivery_id for log in logs]

        # 운영 DB에서 Delivery 객체 사전 조회
        deliveries = self.roscars_session.query(Delivery)\
            .filter(Delivery.delivery_id.in_(delivery_ids)).all()

        delivery_map = {d.delivery_id: d for d in deliveries}
        

        # 관계 맵 구성
        user_map = {
            d.delivery_id: d.user for d in deliveries if d.user is not None
        }
        roscar_map = {
            d.delivery_id: d.roscar for d in deliveries if d.roscar is not None
        }

        # 병합 결과 반환
        return [{
            "event_id": log.event_id,
            "delivery_id": log.delivery_id,
            "timestamp": log.timestamp,
            "event_type": log.new_event.name,
            "user_name": user_map.get(log.delivery_id).user_name if log.delivery_id in user_map else None,
            "roscar_name": roscar_map.get(log.delivery_id).roscar_name if log.delivery_id in roscar_map else None,
            "delivery_status": delivery_map.get(log.delivery_id).delivery_status if log.delivery_id in delivery_map else None,
        } for log in logs]


    # 로봇 주행 이력 + 이름
    def get_precision_stop_result(self):
        logs = self.roscars_log_session.query(PrecisionStopLog)\
            .order_by(PrecisionStopLog.timestamp).all()

        # 로그에 포함된 roscar_id 수집
        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})

        # 운영 DB에서 RosCars 정보 조회
        roscars = self.roscars_session.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        # 병합 결과 구성
        return [{
            "log_id": log.log_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map.get(log.roscar_id).roscar_name if log.roscar_id in roscar_map else None,
            "task_id": log.task_id,
            "is_success": log.is_success,
            "deviation_cm": log.deviation_cm,
            "timestamp": log.timestamp
        } for log in logs]

    def get_roscar_trajectory(self):
        logs = self.roscars_log_session.query(RoscarTrajectoryLog)\
            .order_by(RoscarTrajectoryLog.timestamp).all()

        # 로봇 ID 수집
        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})

        # 운영 DB에서 roscar 정보 조회
        roscars = self.roscars_session.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "trajectory_id": log.trajectory_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map.get(log.roscar_id).roscar_name if log.roscar_id in roscar_map else None,
            "task_id": log.task_id,
            "timestamp": log.timestamp,
            "position_x": log.position_x,
            "position_y": log.position_y,
            "velocity": log.velocity,
            "heading_angle": log.heading_angle,
        } for log in logs]

    def get_roscar_driving_event_log(self):
        logs = self.roscars_log_session.query(RosCarDrivingEventLog)\
            .order_by(RosCarDrivingEventLog.timestamp).all()

        # 필요한 roscar_id 수집
        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})

        # 운영 DB에서 해당 roscar 정보 조회
        roscars = self.roscars_session.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "event_id": log.event_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map.get(log.roscar_id).roscar_name if log.roscar_id in roscar_map else None,
            "driving_event": log.driving_event.name,
            "timestamp": log.timestamp,
        } for log in logs]

    # 학습용 센서 로그 + 로봇 이름
    def get_sensor_for_training(self):
        logs = self.roscars_log_session.query(RoscarSensorFusionRawLog)\
            .order_by(RoscarSensorFusionRawLog.timestamp).all()

        # 필요한 roscar_id 추출
        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})

        # 운영 DB에서 RosCars 정보 조회
        roscars = self.roscars_session.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        # 병합 후 결과 반환
        return [{
            "sensor_log_id": log.sensor_log_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map.get(log.roscar_id).roscar_name if log.roscar_id in roscar_map else None,
            "timestamp": log.timestamp,
            "lidar_raw": log.lidar_raw,
            "imu_data": log.imu_data,
            "ultrasonic_data": log.ultrasonic_data,
            "camera_frame_id": log.camera_frame_id,
        } for log in logs]
    
    def get_control_command_log(self):
        logs = self.roscars_log_session.query(ControlCommandLog)\
            .order_by(ControlCommandLog.timestamp).all()

        # 사용된 roscar_id 추출
        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})

        # 운영 DB에서 RosCars 정보 조회
        roscars = self.roscars_session.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        # 병합 후 결과 반환
        return [{
            "command_id": log.command_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map.get(log.roscar_id).roscar_name if log.roscar_id in roscar_map else None,
            "timestamp": log.timestamp,
            "linear_velocity": log.linear_velocity,
            "angular_velocity": log.angular_velocity,
            "control_source": log.control_source,
        } for log in logs]

    def get_filesystem_log(self):
        logs = self.roscars_log_session.query(FileSystemLog)\
            .order_by(FileSystemLog.timestamp).all()

        # 관련 roscar_id 수집
        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})

        # 운영 DB에서 roscar 정보 조회
        roscars = self.roscars_session.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        # 병합 결과 반환
        return [{
            "log_id": log.log_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map.get(log.roscar_id).roscar_name if log.roscar_id in roscar_map else None,
            "file_path": log.file_path,
            "timestamp": log.timestamp,
        } for log in logs]

    def get_rack_sensor_log(self):
        logs = self.roscars_log_session.query(RackSensorLog)\
            .order_by(RackSensorLog.timestamp).all()

        # Roscar 정보 병합
        roscar_ids = list({log.roscar_id for log in logs if log.roscar_id is not None})
        roscars = self.roscars_session.query(RosCars)\
            .filter(RosCars.roscar_id.in_(roscar_ids)).all()
        roscar_map = {r.roscar_id: r for r in roscars}

        return [{
            "sensor_log_id": log.sensor_log_id,
            "roscar_id": log.roscar_id,
            "roscar_name": roscar_map[log.roscar_id].roscar_name if log.roscar_id in roscar_map else None,
            "rack_id": log.rack_id,
            "rack_status": log.rack_status,
            "rack_position_x": log.rack_position_x,
            "rack_position_y": log.rack_position_y,
            "rack_position_z": log.rack_position_z,
            "timestamp": log.timestamp,
        } for log in logs]
