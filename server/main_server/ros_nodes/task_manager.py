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

            idle_roscars = self.db.query_idle_roscars()

            if not idle_roscars:
                log_info("[TaskManager] No idle roscars available. Task queued.")
                self.db.queue_task(task_id)
                return {"type": "CreateTaskResponse", "success": True, "queued": True}

            best_roscar = self.select_optimal_roscar(idle_roscars)

            assigned = self.assign_task_to_roscar(task_id, best_roscar)

            if assigned:
                log_info(f"[TaskManager] Task {task_id} assigned to Robot {best_roscar['roscar_id']}.")
                return {"type": "CreateTaskResponse", "success": True, "roscar_id": best_roscar["roscar_id"]}
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

    def select_optimal_roscar(self, roscar_list):
        """Idle 로봇 중 하나를 선택한다. (심플: 랜덤 선택)"""
        try:
            selected_roscar = random.choice(roscar_list)
            log_info(f"[TaskManager] Selected Robot: {selected_roscar['roscar_id']}")
            return selected_roscar
        except Exception as e:
            log_error(f"[TaskManager] select_optimal_roscar Error: {str(e)}")
            raise

    def assign_task_to_roscar(self, task_id, roscar):
        """선택된 로봇에게 작업을 할당한다."""
        try:
            self.db.update_roscar_status(roscar["roscar_id"], "WORKING")
            self.db.update_task_status(task_id, "IN_PROGRESS", assigned_roscar_id=roscar["roscar_id"])
            return True
        except Exception as e:
            log_error(f"[TaskManager] assign_task_to_roscar Error: {str(e)}")
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
