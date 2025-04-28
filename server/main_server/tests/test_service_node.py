# test_service_node.py

import sys
import os
import json

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ros_nodes.service_node import MainService

# Mock용 DatabaseAccessor
class MockDBAccessor:
    def search_user_by_name(self, name):
        if name == "test_user":
            return {"id": 1, "name": "test_user", "password": "1234"}
        return None

    def get_user_gui_data(self, user_id):
        return {"layout": "default"}

    def insert_task(self, task_info):
        return 1

    def query_idle_robots(self):
        return [{"robot_id": "robot_001"}]

    def update_robot_status(self, robot_id, status):
        pass

    def update_task_status(self, task_id, status, assigned_robot_id=None):
        pass

    def search_qr_code(self, qr_code):
        if qr_code == "test_qr":
            return {"shoes_id": 101}
        return None

    def search_shoes_data(self, shoes_id):
        if shoes_id == 101:
            return {"size": 270, "color": "black"}
        return None

    def search_robot_status(self):
        return [{"robot_id": "robot_001", "status": "IDLE"}]

# Mock client socket
class MockSocket:
    def sendall(self, data):
        print(f"[MockSocket] Sent: {data.decode('utf-8')}")

def test_login_request(service):
    print("[Test] Sending LoginRequest...")
    message = {
        "type": "LoginRequest",
        "name": "test_user",
        "password": "1234"
    }
    service.route_message(json.dumps(message), MockSocket())

def test_create_task_request(service):
    print("[Test] Sending CreateTaskRequest...")
    message = {
        "type": "CreateTaskRequest",
        "pickup_location": "Station_A",
        "dropoff_location": "Station_B",
        "item_id": "item_001",
        "priority": 1
    }
    service.route_message(json.dumps(message), MockSocket())

def test_qrcode_search_request(service):
    print("[Test] Sending SearchQRCodeData...")
    message = {
        "type": "SearchQRCodeData",
        "qr_code": "test_qr"
    }
    service.route_message(json.dumps(message), MockSocket())

def test_robot_status_request(service):
    print("[Test] Sending RequestRobotStatus...")
    message = {
        "type": "RequestRobotStatus"
    }
    service.route_message(json.dumps(message), MockSocket())

if __name__ == "__main__":
    service = MainService()
    
    # Mock DB로 교체
    service.db = MockDBAccessor()

    test_login_request(service)
    print("-" * 40)
    test_create_task_request(service)
    print("-" * 40)
    test_qrcode_search_request(service)
    print("-" * 40)
    test_robot_status_request(service)
