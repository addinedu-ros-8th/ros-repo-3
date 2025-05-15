# databases/logger.py

from sqlalchemy.orm import Session
from datetime import datetime

from databases.models.roscars_log_models import (
    RosCarEventLog, RosCarEventType,
    TaskEventLog, DeliveryEventLog, PrecisionStopLog,
    DefaultEventType
)

class RoscarsLogWriter:
    def __init__(self, session: Session):
        self.session = session

    def log_roscar_event(self, roscar_id: int, task_id: int | None, event_type: RosCarEventType):
        log = RosCarEventLog(
            roscar_id=roscar_id,
            task_id=task_id,
            event_type=event_type,
            timestamp=datetime.utcnow()
        )
        self._commit_log(log)

    def log_task_event(self, task_id: int, previous: DefaultEventType, current: DefaultEventType):
        log = TaskEventLog(
            task_id=task_id,
            previous_event=previous,
            current_event=current,
            changed_at=datetime.utcnow()
        )
        self._commit_log(log)

    def log_delivery_event(self, delivery_id: int, user_id: int,
                           previous: DefaultEventType, new: DefaultEventType):
        log = DeliveryEventLog(
            delivery_id=delivery_id,
            user_id=user_id,
            previous_event=previous,
            new_event=new,
            timestamp=datetime.utcnow()
        )
        self._commit_log(log)

    def log_delivery_event_failed(self, delivery_id: int, user_id: int):
        log = DeliveryEventLog(
            delivery_id=delivery_id,
            user_id=user_id,
            previous_event=DefaultEventType.PROGRESS_START,
            new_event=DefaultEventType.FAILE,
            timestamp=datetime.utcnow()
        )
        self._commit_log(log)

    def log_task_event_failed(self, task_id: int):
        log = TaskEventLog(
            task_id=task_id,
            previous_event=DefaultEventType.PROGRESS_START,
            current_event=DefaultEventType.FAILE,
            changed_at=datetime.utcnow()
        )
        self._commit_log(log)

    def log_precision_stop_result(self, roscar_id: int, task_id: int,
                                   is_success: bool, deviation_cm: float):
        log = PrecisionStopLog(
            roscar_id=roscar_id,
            task_id=task_id,
            is_success=is_success,
            deviation_cm=deviation_cm,
            timestamp=datetime.utcnow()
        )
        self._commit_log(log)

    def _commit_log(self, log_obj):
        try:
            self.session.add(log_obj)
            self.session.commit()
        except Exception as e:
            self.session.rollback()
            print(f"[LogWriter] Logging failed: {e}")
