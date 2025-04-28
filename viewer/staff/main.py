import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from viewer.staff.dashboard_panel import StaffDashboard
from viewer.staff.request_table import TaskRequestTable
from viewer.staff.theme import apply_staff_theme
from viewer.staff.db_access import fetch_all_tasks
from datetime import datetime
from PyQt5.QtCore import Qt

class StaffGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Staff Dashboard")
        self.setGeometry(100, 100, 1200, 800)

        apply_staff_theme(self)

        central = QWidget(self)
        main_layout = QVBoxLayout()

        # 상단: 로봇 오버뷰 + 요청 폼
        self.dashboard = StaffDashboard()
        main_layout.addWidget(self.dashboard)

        # 하단: 작업 요청 테이블
        self.table = TaskRequestTable()
        main_layout.addWidget(self.table)

        # 요청 제출 시 테이블 업데이트 연결
        self.dashboard.task_submitted.connect(self.table.add_task)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

        # DB에서 작업 불러오고 task_counter 설정
        self._load_existing_tasks()

    def _load_existing_tasks(self):
        tasks = fetch_all_tasks()
        max_id = self._get_max_task_id_number(tasks)
        self.dashboard.task_counter = max_id + 1

        for task in tasks:
            time_obj = task["time"]
            if isinstance(time_obj, datetime):
                time_str = time_obj.strftime("%Y-%m-%d %H:%M:%S")
            else:
                time_str = str(time_obj)

            self.table.add_task({
                "robot": task["robot_id"],
                "task_id": task["task_id"],
                "origin": task["origin"],
                "quantity": task.get("quantity", "-"),
                "status": task["status"],
                "time": time_str
            })

        self.table.sort_by_column(5, Qt.DescendingOrder)

    def _get_max_task_id_number(self, tasks):
        max_id = 0
        for task in tasks:
            task_id = task.get("task_id", "")
            if task_id.startswith("T"):
                try:
                    number = int(task_id[1:])
                    max_id = max(max_id, number)
                except ValueError:
                    continue
        return max_id


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = StaffGUI()
    window.show()
    sys.exit(app.exec_())