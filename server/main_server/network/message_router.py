import json
from databases.utils import MessageUtils
from main_service.main_service import MainService

class MessageRouter:
    def __init__(self):
        self.main_service = MainService()

    def route_message(self, message, client_socket):
        try:
            data = json.loads(message)
            cmd = data.get("Cmd")

            # [1. 로그인 인증 요청] AU
            if cmd == "AU":
                self.main_service.handle_login_request(data, client_socket)

            # [2. 상품 정보 조회 요청] IS
            elif cmd == "IS":
                self.main_service.handle_qrcode_search(data, client_socket)

            # [3. 재고 요청 처리] IR
            elif cmd == "IR":
                self.main_service.handle_inventory_request(data, client_socket)

            # [4-1. 작업 생성 요청] CT
            elif cmd == "CT":
                self.main_service.handle_create_task_request(data, client_socket)

            # [4-2. 작업 취소 요청] CK
            elif cmd == "CK":
                self.main_service.handle_cancel_task(data, client_socket)

            # [6. 작업 완료 여부 확인] TR
            elif cmd == "TR":
                self.main_service.handle_task_result_check(data, client_socket)

            # [5. 로봇 로그 조회 요청] LS
            elif cmd == "LS":
                self.main_service.handle_log_request(data, client_socket)

            # [7. AI 인식 결과 수신] IN
            elif cmd == "IN":
                self.main_service.handle_ai_result(data, client_socket)

            else:
                response = MessageUtils.error("Unknown command", "DL")
                client_socket.sendall(response.encode("utf-8"))

        except Exception:
            response = MessageUtils.error("Server error", "DL")
            client_socket.sendall(response.encode("utf-8"))
