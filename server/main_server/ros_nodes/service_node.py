# service_node.py

import json
from network.emergency_handler import EmergencyHandler
from network.tcp_handler import TCPServer
from db_access import DatabaseAccessor
from task_manager import TaskManager
from logger import log_info, log_error
import yaml

# 설정 읽기
with open("parameters_config.yaml", "r") as f:
    config = yaml.safe_load(f)

SERVER_HOST = config["server"]["host"]
SERVER_PORT = config["server"]["port"]

# MainService 클래스 정의
class MainService:
    def __init__(self):
        self.db = DatabaseAccessor()
        self.task_manager = TaskManager(self.db)
        self.emergency_handler = EmergencyHandler()
        self.tcp_server = TCPServer(SERVER_HOST, SERVER_PORT)
        log_info("[MainService] MainService initialized.")

    def start(self):
        log_info("[MainService] Starting TCP server...")
        self.tcp_server.start_server()

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
            log_error(f"[handle_login_request] {str(e)}")

    def handle_create_task_request(self, data, client_socket):
        try:
            result = self.task_manager.create_task(data)
            client_socket.sendall(json.dumps(result).encode('utf-8'))
        except Exception as e:
            log_error(f"[handle_create_task_request] {str(e)}")

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
            log_error(f"[handle_qrcode_search] {str(e)}")

    def handle_robot_status_request(self, data, client_socket):
        try:
            robot_status = self.db.search_robot_status()
            response = {
                "type": "RobotStatusResponse",
                "robot_status": robot_status
            }
            client_socket.sendall(json.dumps(response).encode('utf-8'))
        except Exception as e:
            log_error(f"[handle_robot_status_request] {str(e)}")

    def emergency_stop_robot(self, robot_id):
        self.emergency_handler.trigger_emergency_stop(robot_id)

    def clear_emergency(self, robot_id):
        self.emergency_handler.clear_emergency(robot_id)

