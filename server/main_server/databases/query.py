from sqlalchemy.orm import Session
from server.main_server.databases.models.roscars_models import RosCars, User, ShoesModel, Task, Delivery, RackLocation, QRCode, ShoesInventory
from typing import Callable, Any

class MainServiceQuery:
    def __init__(self, db_manager, query_function):
        self.db = db_manager
        self.MAX_SENSOR_LOG_COUNT = 100
        self.query_fn = query_function

    def _with_session(self, db_name: str, fn: Callable[[Session], Any], *args, **kwargs) -> Any:
        with self.db.get_session(db_name) as session:
            return fn(session, *args, **kwargs)

    def _with_dual_session(self, fn, *args, **kwargs):
        with self.db.dual_session() as (roscars_sess, log_sess):
            return fn(roscars_sess, log_sess, *args, **kwargs)

    def get_user_by_name(self, user_name: str):
        return self._with_session("roscars", lambda sess: sess.query(User).filter_by(user_name=user_name).first())

    def get_available_roscars(self):
        return self._with_session("roscars", lambda sess: sess.query(RosCars).filter(RosCars.operational_status == 'STANDBY').all())

    def get_task_by_id(self, task_id: int):
        return self._with_session("roscars", lambda sess: sess.query(Task).filter_by(task_id=task_id).first())

    def get_delivery_by_id(self, delivery_id: int):
        return self._with_session("roscars", lambda sess: (
            sess.query(Delivery).filter_by(delivery_id=delivery_id).first()
        ))

    def get_roscar_by_id(self, roscar_id: int):
        return self._with_session("roscars", lambda sess: (
            sess.query(RosCars).filter_by(roscar_id=roscar_id).first()
        ))

    def get_roscar_id_by_name(self, roscar_namespace: str) -> int | None:
        return self._with_session("roscars", lambda sess: (
            (roscar := sess.query(RosCars).filter_by(roscar_namespace=roscar_namespace).first()) and roscar.roscar_id
        ))

    def get_all_locations(self):
        return self._with_session("roscars", lambda sess: (
            sess.query(RackLocation).all()
        ))

    def get_all_users(self):
        return self._with_session("roscars", lambda sess: (
            sess.query(User).all()
        ))

    def get_shoes_model_by_qrcode(self, qr_code_value: str):
        return self._with_session("roscars", lambda sess: (
            sess.query(
                ShoesModel.name,
                ShoesModel.size,
                ShoesModel.color,
                ShoesInventory.quantity,
                RackLocation.name.label("location"),
            ).join(ShoesInventory, ShoesModel.shoes_model_id == ShoesInventory.shoes_model_id)
             .join(QRCode, QRCode.inventory_id == ShoesInventory.inventory_id)
             .join(RackLocation, ShoesInventory.location_id == RackLocation.location_id)
             .filter(QRCode.qr_code_value == qr_code_value).first()
        ))


    def get_model_id_by_name(self, name: str):
        return self._with_session("roscars", lambda sess: (
            m.shoes_model_id if (m := sess.query(ShoesModel).filter_by(name=name).first()) else None
        ))
    
    def get_task_event_log_by_task_id(self, task_id: int):
        return self._with_dual_session(self.query_fn.get_task_event_log_by_task_id, task_id)

    def get_delivery_event_log_by_delivery_id(self, delivery_id: int):
        return self._with_dual_session(self.query_fn.get_delivery_event_log_by_delivery_id, delivery_id)

    def get_all_delivery_logs(self):
        return self._with_dual_session(self.query_fn.get_all_delivery_logs)

    def get_roscar_event_detail(self):
        return self._with_dual_session(self.query_fn.get_roscar_event_detail)


    # 작업 이벤트 로그 + 신발 모델 + 위치 정보
    def get_task_event_history(self):
        return self._with_dual_session(self.query_fn.get_task_event_history)

    # 정밀 정지 결과 + 로봇 이름
    def get_delivery_log(self):
        return self._with_dual_session(self.query_fn.get_delivery_log)

    # 로봇 주행 이력 + 이름
    def get_precision_stop_result(self):
        return self._with_dual_session(self.query_fn.get_precision_stop_result)
    
    def get_roscar_trajectory(self):
        return self._with_dual_session(self.query_fn.get_roscar_trajectory)

    def get_roscar_driving_event_log(self):
        return self._with_dual_session(self.query_fn.get_roscar_driving_event_log)

    def get_sensor_for_training(self):
        return self._with_dual_session(self.query_fn.get_sensor_for_training)

    def get_control_command_log(self):
        return self._with_dual_session(self.query_fn.get_control_command_log)
    
    def get_filesystem_log(self):
        return self._with_dual_session(self.query_fn.get_filesystem_log)

    def get_rack_sensor_log(self):
        return self._with_dual_session(self.query_fn.get_rack_sensor_log)
