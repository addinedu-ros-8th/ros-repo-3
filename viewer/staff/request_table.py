from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt6.QtCore import Qt
from datetime import datetime
from viewer.theme import apply_theme  # 수정된 경로

class TaskRequestTable(QGroupBox):
    def __init__(self):
        super().__init__("My Task Requests")
        apply_theme(self)  # 테마 적용
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout()
        self.table = QTableWidget(0, 6)  # 6열로 확장
        self.table.setHorizontalHeaderLabels(["Robot ID", "Task ID", "Origin(s)", "Quantity", "Status", "Time"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.table)
        self.setLayout(layout)

    def add_task(self, task_info):
        row = self.table.rowCount()
        self.table.insertRow(0)
        self.table.setItem(0, 0, QTableWidgetItem(task_info["robot"]))
        self.table.setItem(0, 1, QTableWidgetItem(task_info["task_id"]))
        self.table.setItem(0, 2, QTableWidgetItem(task_info["origin"]))
        self.table.setItem(0, 3, QTableWidgetItem(task_info.get("quantity", "-")))

        status_item = QTableWidgetItem(task_info["status"])
        status_item.setForeground(Qt.GlobalColor.gray)
        self.table.setItem(0, 4, status_item)

        time_str = (
            task_info["time"].strftime("%Y-%m-%d %H:%M:%S")
            if isinstance(task_info["time"], datetime)
            else str(task_info["time"])
        )

        time_item = QTableWidgetItem(time_str)
        try:
            time_obj = datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S")
            time_item.setData(Qt.UserRole, time_obj)
        except ValueError:
            pass  # 혹시 형식 오류가 있을 경우 대비

        self.table.setItem(0, 5, time_item)

    def sort_by_column(self, column_index, order=Qt.SortOrder.AscendingOrder):
        self.table.sortItems(column_index, order)
