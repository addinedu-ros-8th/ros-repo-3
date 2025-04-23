# request_table.py
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt5.QtCore import Qt
from .theme import apply_worker_theme


class TaskRequestTable(QGroupBox):
    def __init__(self):
        super().__init__("My Task Requests")
        apply_worker_theme(self)
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout()
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Robot ID", "Task ID", "Origin(s)", "Status", "Time"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table)
        self.setLayout(layout)

    def add_task(self, task_info):
        row = self.table.rowCount()
        self.table.insertRow(0)
        self.table.setItem(0, 0, QTableWidgetItem(task_info["robot"]))
        self.table.setItem(0, 1, QTableWidgetItem(task_info["task_id"]))
        self.table.setItem(0, 2, QTableWidgetItem(task_info["origin"]))

        status_item = QTableWidgetItem(task_info["status"])
        status_item.setForeground(Qt.gray)
        self.table.setItem(0, 3, status_item)

        self.table.setItem(0, 4, QTableWidgetItem(task_info["time"]))
