from sqlalchemy.orm import Session
from datetime import datetime

from server.main_server.databases.models.roscars_log_models import (
    RosCarEventLog, RosCarEventType,
    TaskEventLog, DeliveryEventLog, PrecisionStopLog,
    DefaultEventType, 
)

class RoscarsLogWriter:
    def __init__(self, log_session: Session):
        self.log_session = log_session

    def log_roscar_event(self, roscar_id: int, task_id: int | None, event_type: RosCarEventType, camera_angle: int | None = None, commit: bool = True):
        log = RosCarEventLog(
            roscar_id=roscar_id,
            task_id=task_id,
            event_type=event_type,
            timestamp=datetime.utcnow(),
            camera_angle=camera_angle 
        )
        self._commit_log(log, commit)

    def log_task_event(self, task_id: int, previous: DefaultEventType, current: DefaultEventType, commit=True):
        log = TaskEventLog(
            task_id=task_id,
            previous_event=previous,
            current_event=current,
            changed_at=datetime.utcnow()
        )
        self._commit_log(log, commit)

    def log_delivery_event(self, delivery_id: int, user_id: int,
                           previous: DefaultEventType, new: DefaultEventType, commit=True):
        log = DeliveryEventLog(
            delivery_id=delivery_id,
            user_id=user_id,
            previous_event=previous,
            new_event=new,
            timestamp=datetime.utcnow()
        )
        self._commit_log(log, commit)

    def log_delivery_event_failed(self, delivery_id: int, user_id: int, commit=True):
        log = DeliveryEventLog(
            delivery_id=delivery_id,
            user_id=user_id,
            previous_event=DefaultEventType.PROGRESS_START,
            new_event=DefaultEventType.FAILE,
            timestamp=datetime.utcnow()
        )
        self._commit_log(log, commit)

    def log_task_event_failed(self, task_id: int, commit=True):
        log = TaskEventLog(
            task_id=task_id,
            previous_event=DefaultEventType.PROGRESS_START,
            current_event=DefaultEventType.FAILE,
            changed_at=datetime.utcnow()
        )
        self._commit_log(log, commit)

    def log_precision_stop_result(self, roscar_id: int, task_id: int,
                                   is_success: bool, deviation_cm: float, commit=True):
        log = PrecisionStopLog(
            roscar_id=roscar_id,
            task_id=task_id,
            is_success=is_success,
            deviation_cm=deviation_cm,
            timestamp=datetime.utcnow()
        )
        self._commit_log(log, commit)


    def _commit_log(self, log_obj, commit=True):
        try:
            self.log_session.add(log_obj)
            self.log_session.flush()

            if commit:
                self.log_session.commit()
        except Exception as e:
            self.log_session.rollback()
            print(f"[LogWriter] Logging failed: {e}")
