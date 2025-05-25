from server.main_server.databases.models.roscars_log_models import *
from server.main_server.databases.models.roscars_models import RosCars
from datetime import datetime

class LoggerFunction:

    MAX_SENSOR_LOG_COUNT = 100

    @staticmethod
    def build_roscar_event_log(roscar_id, task_id, event_type, camera_angle):
        return RosCarEventLog(
            roscar_id=roscar_id,
            task_id=task_id,
            event_type=event_type,
            timestamp=datetime.utcnow(),
            camera_angle=camera_angle
        )

    @staticmethod
    def build_task_event_log(task_id, previous, current):
        return TaskEventLog(
            task_id=task_id,
            previous_event=previous,
            current_event=current,
            changed_at=datetime.utcnow()
        )

    @staticmethod
    def build_delivery_event_log(delivery_id, user_id, previous, new):
        return DeliveryEventLog(
            delivery_id=delivery_id,
            user_id=user_id,
            previous_event=previous,
            new_event=new,
            timestamp=datetime.utcnow()
        )

    @staticmethod
    def build_delivery_event_failed(delivery_id, user_id):
        return DeliveryEventLog(
            delivery_id=delivery_id,
            user_id=user_id,
            previous_event=DefaultEventType.PROGRESS_START,
            new_event=DefaultEventType.FAILE,
            timestamp=datetime.utcnow()
        )

    @staticmethod
    def build_task_event_failed(task_id):
        return TaskEventLog(
            task_id=task_id,
            previous_event=DefaultEventType.PROGRESS_START,
            current_event=DefaultEventType.FAILE,
            changed_at=datetime.utcnow()
        )

    @staticmethod
    def build_precision_stop_log(roscar_id, task_id, is_success, deviation_cm):
        return PrecisionStopLog(
            roscar_id=roscar_id,
            task_id=task_id,
            is_success=is_success,
            deviation_cm=deviation_cm,
            timestamp=datetime.utcnow()
        )
