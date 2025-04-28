# test_db_access.py

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from db_access import DatabaseAccessor

def test_insert_task():
    try:
        db = DatabaseAccessor()

        # 가짜 task_info (DB 연결 안돼도 호출 자체는 가능해야 함)
        dummy_task_info = {
            "robot_id": "robot_001",
            "task_id": "task_dummy_001",
            "origin": "Station_A",
            "quantity": "2",
            "status": "PENDING"
        }

        print("Testing insert_task() ...")
        try:
            db.insert_task(dummy_task_info)
        except Exception as e:
            print(f"Insert task failed (expected if no DB): {e}")

    except Exception as e:
        print(f"Unexpected error during insert_task test: {e}")

def test_fetch_all_tasks():
    try:
        db = DatabaseAccessor()

        print("Testing fetch_all_tasks() ...")
        try:
            tasks = db.fetch_all_tasks()
            print(f"Fetched tasks: {tasks}")
        except Exception as e:
            print(f"Fetch tasks failed (expected if no DB): {e}")

    except Exception as e:
        print(f"Unexpected error during fetch_all_tasks test: {e}")

if __name__ == "__main__":
    test_insert_task()
    print("-" * 40)
    test_fetch_all_tasks()
