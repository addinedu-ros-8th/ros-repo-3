import json
from network.ros_nodes.service_node import MainService
from databases.log_queries import log_info, log_error

main_service = MainService()

class MessageRouter:
    def __init__(self):
        # 초기화 작업
        pass

    def enqueue(self, msg):
        # msg 송신
        pass

    def dispatch(self):
        # msg 수신
        pass

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

            elif message_type == "RequestRoscarStatus":
                main_service.handle_roscar_status_request(data, client_socket)
            
            elif message_type == "InventoryRequest":
                main_service.handle_inventory_request(data, client_socket)

            elif message_type == "RegisterRoscarRequest":
                main_service.handle_register_roscar(data, client_socket)

            elif message_type == "DeleteRoscarRequest":
                main_service.handle_delete_roscar(data, client_socket)

            elif message_type == "RequestRoscarLocation":
                main_service.handle_location_request(data, client_socket)

            elif message_type == "RequestLogs":
                main_service.handle_log_request(data, client_socket)

            else:
                log_error(f"[MessageRouter] Unknown message type: {message_type}")
                error_response = {
                    "type": "ErrorResponse",
                    "reason": "Unknown message type"
                }
                client_socket.sendall(json.dumps(error_response).encode('utf-8'))

        except Exception as e:
            log_error(f"[MessageRouter] Exception while routing message: {str(e)}")

