import json
from service_node import MainService
from logger import log_info, log_error

# MainService 인스턴스를 전역으로 생성
main_service = MainService()

def route_message(message, client_socket):
    try:
        log_info(f"[MessageRouter] Routing message: {message}")
        
        # 수신한 메시지를 JSON 파싱
        data = json.loads(message)
        message_type = data.get("type")

        if message_type == "LoginRequest":
            main_service.handle_login_request(data, client_socket)

        elif message_type == "CreateTaskRequest":
            main_service.handle_create_task_request(data, client_socket)

        elif message_type == "SearchQRCodeData":
            main_service.handle_qrcode_search(data, client_socket)

        elif message_type == "RequestRobotStatus":
            main_service.handle_roscar_status_request(data, client_socket)

        else:
            log_error(f"[MessageRouter] Unknown message type: {message_type}")
            error_response = {
                "type": "ErrorResponse",
                "reason": "Unknown message type"
            }
            client_socket.sendall(json.dumps(error_response).encode('utf-8'))

    except Exception as e:
        log_error(f"[MessageRouter] Exception while routing message: {str(e)}")
