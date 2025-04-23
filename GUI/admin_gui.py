import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTableWidget, QTableWidgetItem, QGroupBox,
    QDialog, QSizePolicy, QRadioButton, QButtonGroup
)
from PyQt5.QtGui import QFont, QPalette, QColor
from PyQt5.QtCore import Qt

class LogDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Logs")
        self.setGeometry(200, 200, 800, 400)
        layout = QVBoxLayout()
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(["Timestamp", "Robot", "Task", "Event", "Description"])
        layout.addWidget(table)
        self.setLayout(layout)

class RobotDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Management")
        self.setGeometry(200, 200, 800, 400)
        layout = QVBoxLayout()
        table = QTableWidget(0, 4)
        table.setHorizontalHeaderLabels(["Robot ID", "Battery", "Status", "Functionality"])
        layout.addWidget(table)
        self.setLayout(layout)

class RequestDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Requests")
        self.setGeometry(200, 200, 800, 400)
        layout = QVBoxLayout()
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(["Request ID", "Robot", "Task", "Status", "Priority"])
        layout.addWidget(table)
        self.setLayout(layout)

class AdminGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Admin Dashboard")
        self.setGeometry(150, 150, 1400, 900)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f4f2ec;
            }
            QLabel#titleLabel {
                font-size: 22px;
                font-weight: bold;
                color: #3e3e3e;
            }
            QGroupBox {
                border: 1px solid #c2bca2;
                border-radius: 8px;
                margin-top: 10px;
                background-color: #fffdf6;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
                font-weight: bold;
                color: #5b5a4e;
            }
            QPushButton {
                background-color: #708238;
                color: white;
                border-radius: 6px;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #5e6f2a;
            }
            QRadioButton {
                font-size: 14px;
                color: #3e3e3e;
            }
        """)
        self._init_ui()

    def _init_ui(self):
        central = QWidget()
        self.layout = QVBoxLayout()
        self.layout.setSpacing(15)

        # Title
        title = QLabel("Roskatsu System Dashboard")
        title.setObjectName("titleLabel")
        title.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(title)

        # View toggle radio buttons
        self.radio_map = QRadioButton("Map")
        self.radio_sensor = QRadioButton("Sensor Data")
        self.radio_map.setChecked(True)
        view_toggle = QHBoxLayout()
        view_toggle.addWidget(QLabel("Select View:"))
        view_toggle.addWidget(self.radio_map)
        view_toggle.addWidget(self.radio_sensor)
        self.layout.addLayout(view_toggle)

        self.radio_group = QButtonGroup()
        self.radio_group.addButton(self.radio_map)
        self.radio_group.addButton(self.radio_sensor)
        self.radio_map.toggled.connect(self.toggle_views)
        self.radio_sensor.toggled.connect(self.toggle_views)

        # Map Placeholder
        self.map_label = QLabel("Live Map Placeholder")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("background-color: #e7e3d4; border: 2px solid #a89f7d; padding: 10px;")
        self.map_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.map_label.setMinimumHeight(300)
        self.layout.addWidget(self.map_label)

        # Sensor Section
        self.sensor_box = QGroupBox("Sensor Data")
        self.sensor_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sensor_layout = QVBoxLayout()
        self.sensor_graph = QLabel("Sensor Stream Graph Placeholder")
        self.sensor_graph.setAlignment(Qt.AlignCenter)
        self.sensor_graph.setStyleSheet("background-color: #fbf8ee; border: 2px dashed #a3895f; padding: 10px;")
        self.sensor_graph.setMinimumHeight(300)
        sensor_layout.addWidget(self.sensor_graph)
        self.sensor_box.setLayout(sensor_layout)
        self.sensor_box.setVisible(False)
        self.layout.addWidget(self.sensor_box)

        # System Health
        sys_health = QGroupBox("System Health")
        sys_layout = QHBoxLayout()
        sys_layout.addWidget(QLabel("Task States (Idle, Moving, Error...)"))
        sys_layout.addWidget(QLabel("Battery Levels"))
        sys_health.setLayout(sys_layout)
        self.layout.addWidget(sys_health)

        # Management Buttons
        button_box = QGroupBox("Management Panels")
        button_layout = QHBoxLayout()

        btn_robots = QPushButton("Robot Management")
        btn_robots.setMinimumHeight(40)
        btn_robots.clicked.connect(self.open_robot_dialog)

        btn_requests = QPushButton("Requests")
        btn_requests.setMinimumHeight(40)
        btn_requests.clicked.connect(self.open_request_dialog)

        btn_logs = QPushButton("Logs")
        btn_logs.setMinimumHeight(40)
        btn_logs.clicked.connect(self.open_log_dialog)

        button_layout.addWidget(btn_robots)
        button_layout.addWidget(btn_requests)
        button_layout.addWidget(btn_logs)
        button_box.setLayout(button_layout)
        self.layout.addWidget(button_box)

        central.setLayout(self.layout)
        self.setCentralWidget(central)

    def toggle_views(self):
        self.map_label.setVisible(self.radio_map.isChecked())
        self.sensor_box.setVisible(self.radio_sensor.isChecked())

    def open_log_dialog(self):
        self.log_dialog = LogDialog()
        self.log_dialog.exec_()

    def open_robot_dialog(self):
        self.robot_dialog = RobotDialog()
        self.robot_dialog.exec_()

    def open_request_dialog(self):
        self.request_dialog = RequestDialog()
        self.request_dialog.exec_()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AdminGUI()
    window.show()
    sys.exit(app.exec_())