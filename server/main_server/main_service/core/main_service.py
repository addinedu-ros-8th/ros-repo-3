from server.main_server.databases.query import RoscarQuery
from server.main_server.databases.utils import MessageUtils
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.models.roscars_models import ShoesModel, RackLocation, Delivery, DestinationGroup, Task
from server.main_server.databases.models.roscars_log_models import RosCarEventType, DefaultEventType
from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.main_service.core.shutdown import shutdown_flag
import struct

class MainService:
    def __init__(self):
        db = DatabaseManager()
        self.session = db.get_session("roscars")
        self.query = RoscarQuery(self.session)
        self.logger = RoscarsLogWriter(self.session)
        self.enable_shutdown_after_ai_result = False

    # [AU] 로그인 인증 요청
    def handle_login_request(self, req_data, client_socket):
        try:
            user_name = req_data.get("user_name")
            password = req_data.get("password")

            user = self.query.get_user_by_name(user_name)
            if user and user.check_password(password):
                response = {
                    "cmd": "AU",
                    "status": 0x00,
                    "user_id": user.user_id,
                    "user_role": user.user_role,
                }
            else:
                response = {
                    "cmd": "AU",
                    "status": 0x01
                }

            msg = MessageUtils.success(response, "AU")
            client_socket.sendall(msg.encode("utf-8"))

        except Exception:
            err = MessageUtils.error("Login failed", "AU")
            client_socket.sendall(err.encode("utf-8"))

    # [IS] 상품 정보 조회 요청 (QR코드 기반)
    def handle_qrcode_search(self, req_data, client_socket):
        try:
            qr_code_value = req_data.get("qr_code")
            result = self.query.get_shoes_model_by_qrcode(qr_code_value)

            if result:
                response = {
                    "cmd": "IS",
                    "status": 0x00,
                    "name": result.name,
                    "size": result.size,
                    "color": result.color.value,
                    "quantity": result.quantity,
                    "location": result.location
                }
            else:
                response = {
                    "cmd": "IS",
                    "status": 0x01
                }

            msg = MessageUtils.success(response, "IS")
            client_socket.sendall(msg.encode("utf-8"))

        except Exception:
            err = MessageUtils.error("QR 코드 조회 실패", "IS")
            client_socket.sendall(err.encode("utf-8"))

    # [IR] 상품 요청
    def handle_inventory_request(self, req_data, client_socket):
        try:
            user_id = req_data.get("user_id")
            destination_str = req_data.get("destination")
            items = req_data.get("items")

            if not user_id or not items or not isinstance(items, list) or not destination_str:
                response = {"cmd": "IR", "status": 0x01}
                msg = MessageUtils.success(response, "IR")
                client_socket.sendall(msg.encode("utf-8"))
                return

            try:
                destination_enum = DestinationGroup[destination_str]
            except KeyError:
                response = {"cmd": "IR", "status": 0x01}
                msg = MessageUtils.success(response, "IR")
                client_socket.sendall(msg.encode("utf-8"))
                return

            delivery = Delivery(
                user_id=user_id,
                roscar_id=None,
                delivery_status="TO_DO",
                driving_phase_id=None,
                destination=destination_enum
            )
            self.session.add(delivery)
            self.session.flush()

            first_task_id = None
            for item in items:
                sid = item.get("shoes_model_id")
                lid = item.get("location_id")
                qty = item.get("quantity")
                if not sid or not lid or not qty or qty <= 0:
                    continue
                for _ in range(qty):
                    task = Task(
                        delivery_id=delivery.delivery_id,
                        shoes_model_id=sid,
                        location_id=lid,
                        status="TO_DO"
                    )
                    self.session.add(task)
                    self.session.flush()
                    if not first_task_id:
                        first_task_id = task.task_id

            if not first_task_id:
                self.session.rollback()
                response = {"cmd": "IR", "status": 0x01}
            else:
                self.session.commit()
                self.logger.log_delivery_event(
                    delivery_id=delivery.delivery_id,
                    user_id=user_id,
                    previous=DefaultEventType.WAIT,
                    new=DefaultEventType.PROGRESS_START
                )
                response = {
                    "cmd": "IR",
                    "status": 0x00,
                    "delivery_id": delivery.delivery_id,
                    "first_task_id": first_task_id
                }

            msg = MessageUtils.success(response, "IR")
            client_socket.sendall(msg.encode("utf-8"))

        except Exception:
            self.session.rollback()
            response = {"cmd": "IR", "status": 0x01}
            msg = MessageUtils.success(response, "IR")
            client_socket.sendall(msg.encode("utf-8"))

    # [CD] 배송 취소
    def handle_cancel_task(self, req_data, client_socket):
        try:
            user_id = req_data.get("user_id")
            delivery_id = req_data.get("delivery_id")

            if not user_id or not delivery_id:
                response = {"cmd": "CD", "status": 0x01}
                msg = MessageUtils.success(response, "CD")
                client_socket.sendall(msg.encode("utf-8"))
                return

            delivery = self.session.query(Delivery).filter_by(delivery_id=delivery_id).first()
            if not delivery or not delivery.tasks:
                response = {"cmd": "CD", "status": 0x01}
                msg = MessageUtils.success(response, "CD")
                client_socket.sendall(msg.encode("utf-8"))
                return

            first_task = sorted(delivery.tasks, key=lambda t: t.task_id)[0]
            if first_task.status == "DONE":
                response = {"cmd": "CD", "status": 0x01, "delivery_id": delivery_id}
                msg = MessageUtils.success(response, "CD")
                client_socket.sendall(msg.encode("utf-8"))
                return

            delivery.delivery_status = "CANCELLED"
            self.session.commit()

            response = {"cmd": "CD", "status": 0x00, "delivery_id": delivery_id}
            self.logger.log_delivery_event(
                delivery_id=delivery_id,
                user_id=user_id,
                previous=DefaultEventType.PROGRESS_START,
                new=DefaultEventType.CANCEL
            )

            msg = MessageUtils.success(response, "CD")
            client_socket.sendall(msg.encode("utf-8"))

        except Exception:
            self.session.rollback()
            response = {"cmd": "CD", "status": 0x01, "delivery_id": delivery_id}
            msg = MessageUtils.success(response, "CD")
            client_socket.sendall(msg.encode("utf-8"))

    # [TR] 작업 결과 확인 
    def handle_task_result_check(self, req_data, client_socket):
        try:
            user_id = req_data.get("user_id")
            if not user_id:
                response = {"cmd": "TR", "status": 0x01}
                msg = MessageUtils.success(response, "TR")
                client_socket.sendall(msg.encode("utf-8"))
                return

            deliveries = self.session.query(Delivery).filter_by(user_id=user_id).all()
            done_count = sum(task.status == "DONE" for d in deliveries for task in d.tasks)
            in_progress_names = {
                task.shoes_model.name
                for d in deliveries for task in d.tasks
                if task.status == "IN_PROGRESS"
            }

            response = {
                "cmd": "TR",
                "status": 0x00,
                "done_count": done_count,
                "in_progress_count": len(in_progress_names),
                "in_progress_names": list(in_progress_names)
            }
            msg = MessageUtils.success(response, "TR")
            client_socket.sendall(msg.encode("utf-8"))

        except Exception:
            response = {"cmd": "TR", "status": 0x01}
            msg = MessageUtils.success(response, "TR")
            client_socket.sendall(msg.encode("utf-8"))

    # [IN] AI 인식 결과 수신
    def handle_ai_result(self, req_data, client_socket):
        try:
            roscar_id  = req_data.get("roscar_id")
            result_code= req_data.get("result_code")
            if result_code not in (0x00, 0x01):
                raise ValueError("Invalid result_code")

            self.logger.log_roscar_event(
                roscar_id=roscar_id,
                task_id=None,
                event_type=RosCarEventType.OBJECT_DETECTED
            )

            if self.enable_shutdown_after_ai_result:
                shutdown_flag.set()

            response = {"cmd": "IN", "status": 0x00}
        except Exception:
            response = {"cmd": "IN", "status": 0x01}

        msg = MessageUtils.success(response, "IN")
        client_socket.sendall(msg.encode("utf-8"))
