# task_manager.py

import random
from logger import log_info, log_error

class TaskManager:
    def __init__(self, db_accessor):
        self.db = db_accessor
        log_info("[TaskManager] TaskManager initialized.")

    def create_task(self, data):
        try:
            pickup_location = data["pickup_location"]
            dropoff_location = data["dropoff_location"]
            item_id = data["item_id"]
            priority = data.get("priority", 1)

            # 새로운 Task DB에 생성
            task_id = self.db.insert_task({
                "pickup_location": pickup_location,
                "dropoff_location": dropoff_location,
                "item_id": item_id,
                "priority": priority,
                "status": "PENDING"
            })

            log_info(f"[TaskManager] New task created. Task ID: {task_id}")

            # 대기 중인 로봇 검색
            idle_robots = self.db.query_idle_robots()

            if not idle_robots:
                log_info("[TaskManager] No available robots. Task queued.")
                self.db.queue_task(task_id)
                return {"type": "CreateTaskResponse", "success": True, "queued": True}

            # 최적 로봇 선택
            best_robot = self.select_optimal_robot(idle_robots)

            # 작업 할당
            assigned = self.assign_task_to_robot(task_id, best_robot)

            if assigned:
                log_info(f"[TaskManager] Task {task_id} assigned to Robot {best_robot['robot_id']}.")
                return {"type": "CreateTaskResponse", "success": True, "robot_id": best_robot["robot_id"]}
            else:
                log_info(f"[TaskManager] Task {task_id} queued due to assignment failure.")
                self.db.queue_task(task_id)
                return {"type": "CreateTaskResponse", "success": True, "queued": True}

        except Exception as e:
            log_error(f"[TaskManager] create_task Error: {str(e)}")
            return {"type": "CreateTaskResponse", "success": False, "error": str(e)}

    def select_optimal_robot(self, robot_list):
        # (심플 버전) 랜덤으로 하나 고름 - 추후 거리 기반 최적화 가능
        return random.choice(robot_list)

    def assign_task_to_robot(self, task_id, robot):
        try:
            # 여기서 실제 MobileController에 명령 보내야 함 (TODO)
            self.db.update_robot_status(robot["robot_id"], "WORKING")
            self.db.update_task_status(task_id, "IN_PROGRESS", assigned_robot_id=robot["robot_id"])
            return True
        except Exception as e:
            log_error(f"[TaskManager] assign_task_to_robot Error: {str(e)}")
            return False

    def update_task_progress(self, task_id, progress_detail, current_location):
        try:
            self.db.update_task_progress(task_id, progress_detail, current_location)
            log_info(f"[TaskManager] Task {task_id} progress updated.")
        except Exception as e:
            log_error(f"[TaskManager] update_task_progress Error: {str(e)}")

    def complete_task(self, task_id, success=True):
        try:
            if success:
                self.db.update_task_complete(task_id, "COMPLETED")
                log_info(f"[TaskManager] Task {task_id} marked as COMPLETED.")
            else:
                self.db.update_task_complete(task_id, "FAILED")
                log_info(f"[TaskManager] Task {task_id} marked as FAILED.")
        except Exception as e:
            log_error(f"[TaskManager] complete_task Error: {str(e)}")

    def cancel_task(self, task_id):
        try:
            self.db.update_task_status(task_id, "CANCELLED")
            log_info(f"[TaskManager] Task {task_id} cancelled.")
        except Exception as e:
            log_error(f"[TaskManager] cancel_task Error: {str(e)}")
