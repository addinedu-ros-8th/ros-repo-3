from PyQt6.QtWidgets import QDialog, QVBoxLayout, QTableWidget, QHeaderView
from PyQt6.QtWidgets import QTableWidgetItem
from viewer.theme import apply_theme  # 수정된 경로

class LogDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Logs")
        self.setGeometry(200, 200, 800, 400)
        apply_theme(self)

        layout = QVBoxLayout()
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(["Timestamp", "Robot", "Task", "Event", "Description"])
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(table)
        self.setLayout(layout)


class RobotDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Management")
        self.setGeometry(200, 200, 800, 400)
        apply_theme(self)

        layout = QVBoxLayout()
        table = QTableWidget(0, 4)
        table.setHorizontalHeaderLabels(["Robot ID", "Battery", "Status", "Functionality"])
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(table)
        self.setLayout(layout)


class RequestDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Requests")
        self.setGeometry(200, 200, 800, 400)
        apply_theme(self)

        layout = QVBoxLayout()
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(["Request ID", "Robot", "Task", "Status", "Priority"])
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(table)
        self.setLayout(layout)
