from sqlalchemy import func
from sqlalchemy.exc import SQLAlchemyError
from sqlalchemy.dialects.mysql import insert as mysql_insert

from server.main_server.databases.models.roscars_models import RosCars
from server.main_server.databases.models.roscars_log_models import RoscarSensorFusionRawLog

class SystemSchemaFunction:
    MAX_SENSOR_LOG_COUNT = 100

    @staticmethod
    def upsert_roscar_status(db, namespace: str, battery: float, ip: str | None = None):
        with db.get_session("roscars") as session:
            stmt = mysql_insert(RosCars).values(
                roscar_namespace=namespace,
                battery_percentage=battery,
                roscar_ip_v4=ip if ip is not None else "",  # dummy for values only
            )

            if ip is not None and ip != "":
                stmt = stmt.on_duplicate_key_update(
                    battery_percentage=stmt.inserted.battery_percentage,
                    roscar_ip_v4=stmt.inserted.roscar_ip_v4
                )
            else:
                stmt = stmt.on_duplicate_key_update(
                    battery_percentage=stmt.inserted.battery_percentage
                )

            session.execute(stmt)

    @staticmethod
    def save_sensor_fusion_data(db, roscar_namespace: str, timestamp, parsed: dict) -> tuple[bool, int]:
        try:
            with db.dual_session() as (roscars_session, log_session):
                roscar_id = roscars_session.query(RosCars.roscar_id)\
                    .filter_by(roscar_namespace=roscar_namespace)\
                    .scalar()
                if roscar_id is None:
                    return False, -1

                count = log_session.query(func.count(RoscarSensorFusionRawLog.sensor_log_id))\
                    .filter_by(roscar_id=roscar_id).scalar()

                if count >= SystemSchemaFunction.MAX_SENSOR_LOG_COUNT:
                    oldest = log_session.query(RoscarSensorFusionRawLog)\
                        .filter_by(roscar_id=roscar_id)\
                        .order_by(RoscarSensorFusionRawLog.timestamp.asc()).first()
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

                return True, roscar_id

        except SQLAlchemyError:
            return False, roscar_id if 'roscar_id' in locals() else -1
