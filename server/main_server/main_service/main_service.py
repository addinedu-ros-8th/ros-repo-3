from server.main_server.databases.query import RoscarQuery
from server.main_server.databases.utils import MessageUtils
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.models.roscars_models import ShoesModel, RackLocation, Delivery, DestinationGroup, Task
from server.main_server.databases.models.roscars_log_models import RosCarEventType, DefaultEventType
from server.main_server.databases.logger import RoscarsLogWriter

class MainService:
    def __init__(self):
        db = DatabaseManager()
        self.session = db.get_session("roscars")
        self.query = RoscarQuery(self.session)
        self.logger = RoscarsLogWriter(self.session)

    # [AU] 로그인 인증 요청
    def handle_login_request(self, req_data, client_socket):
        try:
            user_name = req_data.get("user_name")
            password = req_data.get("password")

            user = self.query.get_user_by_name(user_name)
            if user and user.check_password(password):
                response = {
                    "cmd": "DL",
                    "status": 0x00,
                    "user_id": user.user_id,
                    "user_role": user.user_role,
                }
            else:
                response = {
                    "cmd": "DL",
                    "status": 0x01
                }

            client_socket.sendall(MessageUtils.success(response, "DL").encode("utf-8"))

        except Exception as e:
            client_socket.sendall(MessageUtils.error("Login failed", "DL").encode("utf-8"))

    # [IS] 상품 정보 조회 요청 (QR코드 기반)
    def handle_qrcode_search(self, req_data, client_socket):
        try:
            qr_code_value = req_data.get("qr_code_value")
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

            client_socket.sendall(MessageUtils.success(response, "IS").encode("utf-8"))

        except Exception as e:
            client_socket.sendall(MessageUtils.error("QR 코드 조회 실패", "IS").encode("utf-8"))

    # [IR] 상품 요청
    def handle_inventory_request(self, req_data, client_socket):
        try:
            user_id = req_data.get("user_id")
            destination_str = req_data.get("destination")  # ex) "G1"
            items = req_data.get("items")

            if not user_id or not items or not isinstance(items, list) or not destination_str:
                response = { "cmd": "IR", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 목적지 Enum 변환
            try:
                destination_enum = DestinationGroup[destination_str]
            except KeyError:
                response = { "cmd": "IR", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 1. Delivery 생성 (destination 포함)
            delivery = Delivery(
                user_id=user_id,
                roscar_id=None,
                delivery_status="TO_DO",
                driving_phase_id=None,
                destination=destination_enum
            )
            self.session.add(delivery)
            self.session.flush()  # delivery_id 확보

            first_task_id = None

            # 2. 장바구니 항목별 Task 생성
            for item in items:
                shoes_model_id = item.get("shoes_model_id")
                location_id = item.get("location_id")
                quantity = item.get("quantity")

                if not shoes_model_id or not location_id or not quantity or quantity <= 0:
                    continue

                shoes_model = self.session.query(ShoesModel).filter_by(shoes_model_id=shoes_model_id).first()
                location = self.session.query(RackLocation).filter_by(location_id=location_id).first()
                if not shoes_model or not location:
                    continue

                for _ in range(quantity):
                    task = Task(
                        delivery_id=delivery.delivery_id,
                        shoes_model_id=shoes_model_id,
                        location_id=location_id,
                        status="TO_DO"
                    )
                    self.session.add(task)
                    self.session.flush()  # task_id 확보

                    if not first_task_id:
                        first_task_id = task.task_id

            if not first_task_id:
                self.session.rollback()
                response = { "cmd": "IR", "status": 0x01 }
            else:
                self.session.commit()

                self.logger.log_delivery_event(  # ✅ commit 이후로 이동
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


            self.logger.log_delivery_event(
                delivery_id=delivery.delivery_id,
                user_id=user_id,
                previous=DefaultEventType.WAIT,
                new=DefaultEventType.PROGRESS_START
                )


        except Exception as e:
            self.session.rollback()
            try:
                if 'delivery' in locals():
                    self.logger.log_delivery_event(
                        delivery_id=delivery.delivery_id,
                        user_id=user_id,
                        previous=DefaultEventType.WAIT,
                        new=DefaultEventType.FAILE
                    )
            except:
                pass  # 로깅 실패 무시
            response = { "cmd": "IR", "status": 0x01 }

        client_socket.sendall(MessageUtils.success(response).encode("utf-8"))

    # [CD] 배송 취소
    def handle_cancel_task(self, req_data, client_socket):
        try:
            user_id = req_data.get("user_id")
            delivery_id = req_data.get("delivery_id")

            if not user_id or not delivery_id:
                response = { "cmd": "CD", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 1. Delivery 및 연결된 Task 조회
            delivery = self.session.query(Delivery).filter_by(delivery_id=delivery_id).first()

            if not delivery:
                response = { "cmd": "CD", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            tasks = delivery.tasks
            if not tasks:
                response = { "cmd": "CD", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 2. 첫 번째 Task 기준 상태 확인
            first_task = sorted(tasks, key=lambda t: t.task_id)[0]

            if first_task.status == "DONE":
                # 취소 불가
                response = { "cmd": "CD", "status": 0x01, "delivery_id": delivery_id }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 3. 상태 변경
            delivery.delivery_status = "CANCELLED"
            self.session.commit()

            # TODO: 4. ROS2 메시지 전송
            # self.ros2_publisher.send_cancel_command(delivery_id=delivery_id)

            response = { "cmd": "CK", "status": 0x00, "delivery_id": delivery_id }

            self.logger.log_delivery_event(
            delivery_id=delivery.delivery_id,
            user_id=user_id,
            previous=DefaultEventType.PROGRESS_START,
            new=DefaultEventType.CANCEL
            )


        except Exception as e:
            self.session.rollback()
            try:
                self.logger.log_delivery_event(
                    delivery_id=delivery_id,
                    user_id=user_id,
                    previous=DefaultEventType.PROGRESS_START,
                    new=DefaultEventType.FAILE
                )
            except:
                pass
            response = { "cmd": "CK", "status": 0x01, "delivery_id": delivery_id }


        client_socket.sendall(MessageUtils.success(response).encode("utf-8"))


    # [TR] 작업 결과 확인 
    def handle_task_result_check(self, req_data, client_socket):
        try:
            user_id = req_data.get("user_id")

            if not user_id:
                response = { "cmd": "TR", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 1. 해당 사용자의 Delivery 목록 조회
            deliveries = self.session.query(Delivery).filter_by(user_id=user_id).all()
            if not deliveries:
                response = { "cmd": "TR", "status": 0x00, "done_count": 0, "in_progress_count": 0, "in_progress_names": [] }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            done_count = 0
            in_progress_names = set()

            for delivery in deliveries:
                for task in delivery.tasks:
                    if task.status == "DONE":
                        done_count += 1
                    elif task.status == "IN_PROGRESS":
                        in_progress_names.add(task.shoes_model.name)

            response = {
                "cmd": "TR",
                "status": 0x00,
                "done_count": done_count,
                "in_progress_count": len(in_progress_names),
                "in_progress_names": list(in_progress_names)
            }

        except Exception as e:
            response = { "cmd": "TR", "status": 0x01 }

        client_socket.sendall(MessageUtils.success(response).encode("utf-8"))

    # [IN] AI 인식 결과 수신
    def handle_ai_result(self, req_data, client_socket):
        try:
            roscar_id = req_data.get("roscar_id")
            result_code = req_data.get("result_code")

            if roscar_id is None or result_code is None:
                response = { "cmd": "IN", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 인식 결과 매핑
            OBJECT_TYPE_MAP = {
                0x00: "Roscar",
                0x01: "Person",
            }
            detected_object = OBJECT_TYPE_MAP.get(result_code, "Unknown")

            # 인식 결과 해석
            # ROS2 비상 정지 명령 전송 예시
            # self.ros2_publisher.send_emergency_stop(roscar_id)
            # self.ros2_publisher.send_pause(roscar_id)
            # self.ros2_publisher.send_warning(roscar_id)

            self.logger.log_roscar_event(
            roscar_id=roscar_id,
            task_id=None,
            event_type=RosCarEventType.OBJECT_DETECTED
            )


            response = { "cmd": "IN", "status": 0x00 }

        except Exception as e:
            try:
                self.logger.log_roscar_event(
                    roscar_id=roscar_id if 'roscar_id' in locals() else -1,
                    task_id=None,
                    event_type=RosCarEventType.EMERGENCY_STOP  # 또는 FAIL용 커스텀 타입이 있다면 그것으로
                )
            except:
                pass
            response = { "cmd": "IN", "status": 0x01 }

        client_socket.sendall(MessageUtils.success(response).encode("utf-8"))

