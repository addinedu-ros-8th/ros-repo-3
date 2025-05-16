import json
from rclpy.node import Node
from sqlalchemy import func
from sqlalchemy.exc import SQLAlchemyError
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.models.roscars_models import RosCars
from server.main_server.databases.models.roscars_log_models import RoscarSensorFusionRawLog

db = DatabaseManager()
MAX_SENSOR_LOG_COUNT = 100


class SensorUtils(Node):
    def __init__(self, logger, db_manager: DatabaseManager = db):
        super().__init__('sensor_utils')
        self.logger = logger
        self.db = db_manager

    def parse_sensor_data(self, msg):
        """SensorData 메시지에서 센서 raw 데이터 파싱"""
        return {
            "lidar": json.loads(msg.lidar_raw),
            "imu": json.loads(msg.imu_data),
            "ultra": json.loads(msg.ultrasonic_data),
        }

    def save_sensor_data_to_db(self, roscar_name, timestamp, parsed):
        """센서 데이터 저장 및 오래된 로그 정리"""
        roscars_session = self.db.get_session("roscars")
        log_session = self.db.get_session("roscars_log")

        try:
            roscar_id = roscars_session.query(RosCars.roscar_id)\
                .filter(RosCars.roscar_name == roscar_name)\
                .scalar()

            if roscar_id is None:
                self.logger.warning(f"RosCars 테이블에 '{roscar_name}' 없음 → 저장 건너뜀")
                return

            count = log_session.query(func.count(RoscarSensorFusionRawLog.sensor_log_id))\
                .filter(RoscarSensorFusionRawLog.roscar_id == roscar_id)\
                .scalar()

            if count >= MAX_SENSOR_LOG_COUNT:
                oldest = log_session.query(RoscarSensorFusionRawLog)\
                    .filter(RoscarSensorFusionRawLog.roscar_id == roscar_id)\
                    .order_by(RoscarSensorFusionRawLog.timestamp.asc())\
                    .first()
                if oldest:
                    log_session.delete(oldest)

            log = RoscarSensorFusionRawLog(
                roscar_id=roscar_id,
                timestamp=timestamp,
                lidar_raw=parsed["lidar"],
                imu_data=parsed["imu"],
                ultrasonic_data=parsed["ultra"],
                camera_frame_id=None
            )
            log_session.add(log)
            log_session.commit()
            self.logger.info(f"센서 로그 저장 완료: {roscar_name}")

        except SQLAlchemyError as e:
            log_session.rollback()
            self.logger.error(f"센서 로그 저장 실패: {e}")

        finally:
            roscars_session.close()
            log_session.close()


class MessageUtils:
    @staticmethod
    def success(data: dict, type_: str):
        return json.dumps({
            "type": type_,
            "success": True,
            "data": data
        })

    @staticmethod
    def error(reason: str, type_: str = "ErrorResponse"):
        return json.dumps({
            "type": type_,
            "success": False,
            "reason": reason
        })

