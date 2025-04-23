# dialogs.py

from PyQt5.QtWidgets import QDialog, QVBoxLayout, QTableWidget, QHeaderView
from PyQt5.QtWidgets import QTableWidgetItem
from theme import apply_kaki_theme


class LogDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Logs")
        self.setGeometry(200, 200, 800, 400)
        apply_kaki_theme(self)

        layout = QVBoxLayout()
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(["Timestamp", "Robot", "Task", "Event", "Description"])
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(table)
        self.setLayout(layout)


class RobotDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Management")
        self.setGeometry(200, 200, 800, 400)
        apply_kaki_theme(self)

        layout = QVBoxLayout()
        table = QTableWidget(0, 4)
        table.setHorizontalHeaderLabels(["Robot ID", "Battery", "Status", "Functionality"])
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(table)
        self.setLayout(layout)


class RequestDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Requests")
        self.setGeometry(200, 200, 800, 400)
        apply_kaki_theme(self)

        layout = QVBoxLayout()
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(["Request ID", "Robot", "Task", "Status", "Priority"])
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(table)
        self.setLayout(layout)
