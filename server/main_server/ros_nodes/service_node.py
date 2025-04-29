import os
import json
import yaml
from network.emergency_handler import EmergencyHandler
from network.tcp_handler import TCPServer
from db_access import DatabaseAccessor
from ros_nodes.task_manager import TaskManager
from logger import log_info, log_error

# 서버 설정 파일 경로 안전하게 읽기
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "../parameters_config.yaml")

with open(CONFIG_PATH, "r") as f:
    config = yaml.safe_load(f)

SERVER_HOST = config["server"]["host"]
SERVER_PORT = config["server"]["port"]

class MainService:
    def __init__(self):
        self.db = DatabaseAccessor()
        self.task_manager = TaskManager(self.db)
        self.emergency_handler = EmergencyHandler()
        self.tcp_server = TCPServer(SERVER_HOST, SERVER_PORT, self)

        log_info("[MainService] Initialized.")

    def start(self):
        log_info("[MainService] Starting TCP server...")
        self.tcp_server.start_server()

    def route_message(self, message, client_socket):
        """받은 message를 타입에 따라 라우팅"""
        try:
            log_info(f"[MainService] Routing message: {message}")
            
            # 메시지 파싱
            if message.startswith("POST /add_robot"):
                # 'Content-Length' 헤더의 길이를 확인해서 본문을 정확히 파싱합니다.
                content_length = int(message.split("Content-Length:")[1].split("\r\n")[0].strip())
                json_data = message.split("\r\n\r\n")[1][:content_length]

                # JSON 데이터 파싱
                data = json.loads(json_data)
                message_type = data.get("type")

                if message_type == "AddRobotRequest":
                    self.handle_add_robot_request(data, client_socket)
                else:
                    self.send_error_response(client_socket, "Unknown message type")

        except json.JSONDecodeError:
            log_error("[MainService] Failed to parse incoming message as JSON.")
            self.send_error_response(client_socket, "Invalid JSON format")
        except Exception as e:
            log_error(f"[MainService] route_message Exception: {str(e)}")
            self.send_error_response(client_socket, "Internal server error")

    def send_error_response(self, client_socket, reason):
        """클라이언트에 에러 응답 전송"""
        response = {
            "type": "ErrorResponse",
            "reason": reason
        }
        client_socket.sendall(json.dumps(response).encode('utf-8'))

    def handle_login_request(self, data, client_socket):
        try:
            name = data["name"]
            password = data["password"]
            user_info = self.db.search_user_by_name(name)

            if user_info and user_info["password"] == password:
                log_info(f"[Login] {name} login successful.")
                gui_data = self.db.get_user_gui_data(user_info["id"])
                response = {
                    "type": "LoginResponse",
                    "success": True,
                    "gui_data": gui_data
                }
            else:
                log_info(f"[Login] {name} login failed.")
                response = {
                    "type": "LoginResponse",
                    "success": False
                }

            client_socket.sendall(json.dumps(response).encode('utf-8'))
        except Exception as e:
            log_error(f"[MainService] handle_login_request Error: {str(e)}")
            self.send_error_response(client_socket, "Login failed")

    def handle_create_task_request(self, data, client_socket):
        try:
            result = self.task_manager.create_task(data)
            client_socket.sendall(json.dumps(result).encode('utf-8'))
        except Exception as e:
            log_error(f"[MainService] handle_create_task_request Error: {str(e)}")
            self.send_error_response(client_socket, "Task creation failed")

    def handle_qrcode_search(self, data, client_socket):
        try:
            qr_code = data["qr_code"]
            qr_info = self.db.search_qr_code(qr_code)
            if qr_info:
                shoes_info = self.db.search_shoes_data(qr_info["shoes_id"])
                if shoes_info:
                    response = {
                        "type": "ShoesInfoResponse",
                        "success": True,
                        "shoes_info": shoes_info
                    }
                else:
                    response = {
                        "type": "ShoesInfoSearchFailed",
                        "reason": "Shoes data missing"
                    }
            else:
                response = {
                    "type": "QRCodeSearchFailed",
                    "reason": "QR not found"
                }
            client_socket.sendall(json.dumps(response).encode('utf-8'))

        except Exception as e:
            log_error(f"[MainService] handle_qrcode_search Error: {str(e)}")
            self.send_error_response(client_socket, "QR code search failed")

    def handle_robot_status_request(self, data, client_socket):
        try:
            robot_status = self.db.search_robot_status()
            response = {
                "type": "RobotStatusResponse",
                "robot_status": robot_status
            }
            client_socket.sendall(json.dumps(response).encode('utf-8'))
        except Exception as e:
            log_error(f"[MainService] handle_robot_status_request Error: {str(e)}")
            self.send_error_response(client_socket, "Robot status request failed")

    def handle_add_robot_request(self, data, client_socket):
        """로봇 추가 요청을 처리하는 메서드"""
        try:
            robot_name = data.get("robot_name")
            robot_ip = data.get("robot_ip")

            if not robot_name or not robot_ip:
                log_error(f"[MainService] Invalid robot data received: {data}")
                self.send_error_response(client_socket, "Invalid robot data")
                return

            # 로봇 정보를 DB에 저장
            robot_id = self.db.add_robot(robot_name, robot_ip)
            
            # 성공 응답 보내기
            response = {
                "type": "AddRobotResponse",
                "message": f"Robot {robot_name} added successfully!",
                "robot_id": robot_id
            }
            
            # 응답 로그 추가
            log_info(f"[MainService] Sending response: {json.dumps(response)}")

            client_socket.sendall(json.dumps(response).encode('utf-8'))

        except Exception as e:
            log_error(f"[MainService] handle_add_robot_request Error: {str(e)}")
            self.send_error_response(client_socket, "Robot addition failed")

    def emergency_stop_robot(self, robot_id):
        self.emergency_handler.trigger_emergency_stop(robot_id)

    def clear_emergency(self, robot_id):
        self.emergency_handler.clear_emergency(robot_id)
