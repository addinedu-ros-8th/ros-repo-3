from databases.query import RoscarQuery
from databases.utils import MessageUtils
from databases.database_manager import DatabaseManager
from databases.models.roscars_models import ShoesModel, RackLocation, Delivery, Task

class MainService:
    def __init__(self):
        db = DatabaseManager()
        self.session = db.get_session("roscars")
        self.query = RoscarQuery(self.session)

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
            items = req_data.get("items")  # List of dicts: [{shoes_model_id, location_id, quantity}, ...]

            if not user_id or not items or not isinstance(items, list):
                response = { "cmd": "IR", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 1. Delivery 생성
            delivery = Delivery(
                user_id=user_id,
                roscar_id=None,
                delivery_status="TO_DO",
                driving_phase_id=None
            )
            self.session.add(delivery)
            self.session.flush()  # delivery_id 확보

            first_task_id = None

            # 2. 각 장바구니 항목 처리
            for item in items:
                shoes_model_id = item.get("shoes_model_id")
                location_id = item.get("location_id")
                quantity = item.get("quantity")

                if not shoes_model_id or not location_id or not quantity or quantity <= 0:
                    continue  # 유효하지 않으면 건너뜀

                # 유효성 검사
                shoes_model = self.session.query(ShoesModel).filter_by(shoes_model_id=shoes_model_id).first()
                location = self.session.query(RackLocation).filter_by(location_id=location_id).first()
                if not shoes_model or not location:
                    continue  # 유효하지 않으면 해당 항목 무시

                # 수량만큼 Task 생성
                for _ in range(quantity):
                    task = Task(
                        delivery_id=delivery.delivery_id,
                        shoes_model_id=shoes_model_id,
                        location_id=location_id,
                        status="TO_DO"
                    )
                    self.session.add(task)
                    self.session.flush()
                    if not first_task_id:
                        first_task_id = task.task_id

            # Task 1개 이상 생성되지 않았다면 롤백
            if not first_task_id:
                self.session.rollback()
                response = { "cmd": "IR", "status": 0x01 }
            else:
                self.session.commit()
                response = {
                    "cmd": "IR",
                    "status": 0x00,
                    "delivery_id": delivery.delivery_id,
                    "first_task_id": first_task_id
                }

        except Exception as e:
            self.session.rollback()
            response = { "cmd": "IR", "status": 0x01 }

        client_socket.sendall(MessageUtils.success(response).encode("utf-8"))


    # [CT] 작업 생성
    def handle_create_task_request(self, req_data, client_socket):
        pass

    # [CK] 작업 취소
    def handle_cancel_task(self, req_data, client_socket):
        """현재 작업 취소 요청 처리"""
        pass

    # [TR] 작업 결과 확인
    def handle_task_result_check(self, req_data, client_socket):
        """작업 완료 여부 확인 요청"""
        pass

    # [LS] 로그 조회 요청
    def handle_log_request(self, req_data, client_socket):
        """로봇 이벤트 로그 등 조회 요청"""
        pass

    # [IN] AI 인식 결과 수신
    def handle_ai_result(self, req_data, client_socket):
        """AI 서버에서 전송된 인식 결과 수신"""
        pass
