# task_manager.py

import random
from logger import log_info, log_error

class TaskManager:
    def __init__(self, db_accessor):
        self.db = db_accessor
        log_info("[TaskManager] Initialized.")

    def create_task(self, data):
        """새로운 작업을 생성하고 로봇에 할당하거나 큐에 저장한다."""
        try:
            pickup_location = data["pickup_location"]
            dropoff_location = data["dropoff_location"]
            item_id = data["item_id"]
            priority = data.get("priority", 1)

            task_info = {
                "pickup_location": pickup_location,
                "dropoff_location": dropoff_location,
                "item_id": item_id,
                "priority": priority,
                "status": "PENDING"
            }

            task_id = self.db.insert_task(task_info)
            log_info(f"[TaskManager] Task created. Task ID: {task_id}")

            idle_robots = self.db.query_idle_robots()

            if not idle_robots:
                log_info("[TaskManager] No idle robots available. Task queued.")
                self.db.queue_task(task_id)
                return {"type": "CreateTaskResponse", "success": True, "queued": True}

            best_robot = self.select_optimal_robot(idle_robots)

            assigned = self.assign_task_to_robot(task_id, best_robot)

            if assigned:
                log_info(f"[TaskManager] Task {task_id} assigned to Robot {best_robot['robot_id']}.")
                return {"type": "CreateTaskResponse", "success": True, "robot_id": best_robot["robot_id"]}
            else:
                log_info(f"[TaskManager] Assignment failed. Task {task_id} queued.")
                self.db.queue_task(task_id)
                return {"type": "CreateTaskResponse", "success": True, "queued": True}

        except KeyError as e:
            log_error(f"[TaskManager] Missing key in task data: {e}")
            return {"type": "CreateTaskResponse", "success": False, "error": f"Missing key: {e}"}
        except Exception as e:
            log_error(f"[TaskManager] create_task Error: {str(e)}")
            return {"type": "CreateTaskResponse", "success": False, "error": str(e)}

    def select_optimal_robot(self, robot_list):
        """Idle 로봇 중 하나를 선택한다. (심플: 랜덤 선택)"""
        try:
            selected_robot = random.choice(robot_list)
            log_info(f"[TaskManager] Selected Robot: {selected_robot['robot_id']}")
            return selected_robot
        except Exception as e:
            log_error(f"[TaskManager] select_optimal_robot Error: {str(e)}")
            raise

    def assign_task_to_robot(self, task_id, robot):
        """선택된 로봇에게 작업을 할당한다."""
        try:
            self.db.update_robot_status(robot["robot_id"], "WORKING")
            self.db.update_task_status(task_id, "IN_PROGRESS", assigned_robot_id=robot["robot_id"])
            return True
        except Exception as e:
            log_error(f"[TaskManager] assign_task_to_robot Error: {str(e)}")
            return False

    def update_task_progress(self, task_id, progress_detail, current_location):
        """작업 진행 상황을 업데이트한다."""
        try:
            self.db.update_task_progress(task_id, progress_detail, current_location)
            log_info(f"[TaskManager] Task {task_id} progress updated.")
        except Exception as e:
            log_error(f"[TaskManager] update_task_progress Error: {str(e)}")

    def complete_task(self, task_id, success=True):
        """작업을 완료 또는 실패로 처리한다."""
        try:
            status = "COMPLETED" if success else "FAILED"
            self.db.update_task_complete(task_id, status)
            log_info(f"[TaskManager] Task {task_id} marked as {status}.")
        except Exception as e:
            log_error(f"[TaskManager] complete_task Error: {str(e)}")

    def cancel_task(self, task_id):
        """작업을 취소한다."""
        try:
            self.db.update_task_status(task_id, "CANCELLED")
            log_info(f"[TaskManager] Task {task_id} cancelled.")
        except Exception as e:
            log_error(f"[TaskManager] cancel_task Error: {str(e)}")
