from sqlalchemy import func
from sqlalchemy.exc import SQLAlchemyError

from db.roscars_models import RosCars
from db.roscars_log_models import SensorFusionRawLog
from db.connect_db import get_roscars_session, get_roscars_log_session

MAX_SENSOR_LOG_COUNT = 100

def save_sensor_data_to_db(robot_name, timestamp, parsed, logger):
    roscars_session = get_roscars_session()
    log_session = get_roscars_log_session()

    try:
        roscar_id = roscars_session.query(RosCars.roscar_id)\
            .filter(RosCars.roscar_name == robot_name)\
            .scalar()

        if roscar_id is None:
            logger.warn(f"RosCars 테이블에 '{robot_name}' 이름 없음 → 저장 건너뜀")
            return

        count = log_session.query(func.count(SensorFusionRawLog.sensor_log_id))\
            .filter(SensorFusionRawLog.roscar_id == roscar_id)\
            .scalar()

        if count >= MAX_SENSOR_LOG_COUNT:
            oldest_log = log_session.query(SensorFusionRawLog)\
                .filter(SensorFusionRawLog.roscar_id == roscar_id)\
                .order_by(SensorFusionRawLog.timestamp.asc())\
                .first()
            if oldest_log:
                log_session.delete(oldest_log)

        log = SensorFusionRawLog(
            roscar_id=roscar_id,
            timestamp=timestamp,
            lidar_raw=parsed["lidar"],
            imu_data=parsed["imu"],
            ultrasonic_data=parsed["ultra"],
            camera_frame_id=None
        )
        log_session.add(log)
        log_session.commit()
        logger.info("SensorData → DB 저장 완료")

    except SQLAlchemyError as e:
        log_session.rollback()
        logger.error(f"SensorData DB 저장 실패: {e}")
    finally:
        roscars_session.close()
        log_session.close()
