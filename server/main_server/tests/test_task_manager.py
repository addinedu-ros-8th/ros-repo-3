# test_task_manager.py

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ros_nodes.task_manager import TaskManager

# Mock용 DatabaseAccessor
class MockDBAccessor:
    def __init__(self):
        self.tasks = []
        self.roscars = [{"roscar_id": "roscar_001", "status": "IDLE"}]

    def insert_task(self, task_info):
        self.tasks.append(task_info)
        return len(self.tasks)  # task_id 대신 인덱스 반환 (더미)

    def query_idle_roscars(self):
        return self.roscars

    def update_roscar_status(self, roscar_id, status):
        for roscar in self.roscars:
            if roscar["roscar_id"] == roscar_id:
                roscar["status"] = status

    def update_task_status(self, task_id, status, assigned_roscar_id=None):
        if 0 < task_id <= len(self.tasks):
            self.tasks[task_id - 1]["status"] = status
            self.tasks[task_id - 1]["assigned_roscar_id"] = assigned_roscar_id

    def queue_task(self, task_id):
        # 실제 큐잉은 생략하고 로그만 남긴다고 가정
        print(f"MockDB: Task {task_id} queued.")

def test_create_task_success():
    try:
        mock_db = MockDBAccessor()
        task_manager = TaskManager(mock_db)

        dummy_task_data = {
            "pickup_location": "Station_A",
            "dropoff_location": "Station_B",
            "item_id": "item_001",
            "priority": 1
        }

        print("Testing create_task() with available roscar ...")
        result = task_manager.create_task(dummy_task_data)
        print(f"Result: {result}")
    except Exception as e:
        print(f"Unexpected error in test_create_task_success: {e}")

def test_create_task_no_idle_roscar():
    try:
        mock_db = MockDBAccessor()
        mock_db.roscars = []  # Idle 로봇 없음 설정
        task_manager = TaskManager(mock_db)

        dummy_task_data = {
            "pickup_location": "Station_A",
            "dropoff_location": "Station_B",
            "item_id": "item_002",
            "priority": 1
        }

        print("Testing create_task() with no idle roscar (should queue task) ...")
        result = task_manager.create_task(dummy_task_data)
        print(f"Result: {result}")
    except Exception as e:
        print(f"Unexpected error in test_create_task_no_idle_roscar: {e}")

if __name__ == "__main__":
    test_create_task_success()
    print("-" * 40)
    test_create_task_no_idle_roscar()
