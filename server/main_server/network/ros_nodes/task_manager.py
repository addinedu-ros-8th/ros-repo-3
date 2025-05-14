from database.models import Task, Delivery
from sqlalchemy.orm import Session

class TaskManager:
    def __init__(self, db_session: Session):
        self.db = db_session

    def create_task(self, item_id, from_loc, to_loc):
        # DB에 Task, Delivery insert
        pass

    def assign_task(self, task_id):
        # 로봇 상태 확인 후 Task 할당
        pass

    def update_progress(self, task_id, progress):
        pass

    def complete_task(self, task_id):
        pass
