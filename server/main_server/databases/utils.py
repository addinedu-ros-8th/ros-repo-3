import json
from rclpy.node import Node
from sqlalchemy import func
from sqlalchemy.exc import SQLAlchemyError

from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.models.roscars_models import RosCars
from server.main_server.databases.models.roscars_log_models import (
    RoscarSensorFusionRawLog,
    RosCarEventType
)
from server.main_server.databases.logger import RoscarsLogWriter

MAX_SENSOR_LOG_COUNT = 100

class SensorUtils(Node):
    def __init__(self, db_logger: RoscarsLogWriter, db_manager: DatabaseManager):
        super().__init__('sensor_utils')
        self.db_logger = db_logger
        self.db = db_manager

    def parse_sensor_data(self, msg):
        """SensorData 메시지에서 센서 raw 데이터 파싱"""
        return {
            "lidar": json.loads(msg.lidar_raw),
            "imu": json.loads(msg.imu_data),
            "ultra": json.loads(msg.ultrasonic_data),
        }

    def save_sensor_data_to_db(self, roscar_namespace: str, timestamp, parsed: dict):
        """센서 데이터 저장 및 오래된 로그 정리"""
        try:
            with self.db.session_scope("roscars") as roscars_session, \
                 self.db.session_scope("roscars_log") as log_session:

                # 운영 DB에서 roscar_id 조회
                roscar_id = roscars_session.query(RosCars.roscar_id)\
                    .filter(RosCars.roscar_namespace == roscar_namespace)\
                    .scalar()

                if roscar_id is None:
                    self.db_logger.log_roscar_event(
                        roscar_id=-1,
                        task_id=None,
                        event_type=RosCarEventType.INVALID_ROSCAR_NAME
                    )
                    return

                # 로그 수 제한 초과 시 삭제
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

                # 새 로그 저장
                log = RoscarSensorFusionRawLog(
                    roscar_id=roscar_id,
                    timestamp=timestamp,
                    lidar_raw=parsed["lidar"],
                    imu_data=parsed["imu"],
                    ultrasonic_data=parsed["ultra"],
                    camera_frame_id=None
                )
                log_session.add(log)

                self.db_logger.log_roscar_event(
                    roscar_id=roscar_id,
                    task_id=None,
                    event_type=RosCarEventType.SENSOR_DATA_SAVED
                )

        except SQLAlchemyError:
            self.db_logger.log_roscar_event(
                roscar_id=roscar_id if 'roscar_id' in locals() else -1,
                task_id=None,
                event_type=RosCarEventType.SENSOR_SAVE_FAILED
            )


class MessageUtils:
    @staticmethod
    def success(data: dict):
        return json.dumps({
            "type": data.get("cmd", "Response"),
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
