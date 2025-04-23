# task_control_panel.py
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QTableWidget, QHeaderView
from PyQt5.QtWidgets import QTableWidgetItem


def apply_kaki_theme(widget):
    widget.setStyleSheet("""
        QDialog {
            background-color: #f4f2ec;
        }
        QTableWidget {
            background-color: #fffdf6;
            color: #3e3e3e;
            gridline-color: #a89f7d;
        }
        QHeaderView::section {
            background-color: #d2c8a9;
            color: #3e3e3e;
            font-weight: bold;
            padding: 6px;
            border: 1px solid #c2bca2;
        }
    """)


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
