# dashboard_panel.py

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QSizePolicy, QComboBox, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import Qt
from .theme import apply_kaki_theme


class MonitorPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.selected_robot = "Pinky 1"
        apply_kaki_theme(self)
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(15)

        # 제목
        title = QLabel("Roskatsu System Dashboard")
        title.setObjectName("titleLabel")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # 로봇 선택 & 뷰 전환
        selection_row = QHBoxLayout()
        robot_label = QLabel("Select Robot:")
        robot_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self.robot_combo = QComboBox()
        self.robot_combo.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.robot_combo.addItems(["Pinky 1", "Pinky 2"])
        self.robot_combo.currentTextChanged.connect(self.update_robot_selection)

        selection_row.addWidget(robot_label)
        selection_row.addWidget(self.robot_combo)
        selection_row.addSpacing(30)

        selection_row.addWidget(QLabel("Select View:"))
        self.radio_map = QRadioButton("Map")
        self.radio_sensor = QRadioButton("Sensor Data")
        self.radio_map.setChecked(True)
        selection_row.addWidget(self.radio_map)
        selection_row.addWidget(self.radio_sensor)

        self.radio_group = QButtonGroup()
        self.radio_group.addButton(self.radio_map)
        self.radio_group.addButton(self.radio_sensor)
        self.radio_map.toggled.connect(self.toggle_views)
        self.radio_sensor.toggled.connect(self.toggle_views)

        layout.addLayout(selection_row)

        # 맵 라벨
        self.map_label = QLabel()
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("background-color: #e7e3d4; border: 2px solid #a89f7d; padding: 10px;")
        self.map_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.map_label.setMinimumHeight(300)
        layout.addWidget(self.map_label)

        # 센서 뷰
        self.sensor_box = QGroupBox("Sensor Data")
        self.sensor_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sensor_layout = QVBoxLayout()
        self.sensor_graph = QLabel()
        self.sensor_graph.setAlignment(Qt.AlignCenter)
        self.sensor_graph.setStyleSheet("background-color: #fbf8ee; border: 2px dashed #a3895f; padding: 10px;")
        self.sensor_graph.setMinimumHeight(300)
        sensor_layout.addWidget(self.sensor_graph)
        self.sensor_box.setLayout(sensor_layout)
        self.sensor_box.setVisible(False)
        layout.addWidget(self.sensor_box)

        # 시스템 상태
        self.sys_health = QGroupBox("System Health")
        self.sys_layout = QHBoxLayout()
        self.health_label = QLabel()
        self.sys_layout.addWidget(self.health_label)
        self.sys_health.setLayout(self.sys_layout)
        layout.addWidget(self.sys_health)

        # 버튼 영역
        button_box = QGroupBox("Management Panels")
        button_layout = QHBoxLayout()

        self.btn_robots = QPushButton("Robot Management")
        self.btn_robots.setMinimumHeight(40)

        self.btn_requests = QPushButton("Requests")
        self.btn_requests.setMinimumHeight(40)

        self.btn_logs = QPushButton("Logs")
        self.btn_logs.setMinimumHeight(40)

        button_layout.addWidget(self.btn_robots)
        button_layout.addWidget(self.btn_requests)
        button_layout.addWidget(self.btn_logs)
        button_box.setLayout(button_layout)
        layout.addWidget(button_box)

        self.setLayout(layout)
        self.refresh_robot_view()

    def update_robot_selection(self, text):
        self.selected_robot = text
        self.refresh_robot_view()

    def refresh_robot_view(self):
        self.map_label.setText(f"Live Map for {self.selected_robot}")
        self.sensor_graph.setText(f"Sensor Stream Graph for {self.selected_robot}")
        self.health_label.setText(f"System Health: {self.selected_robot} status display")

    def toggle_views(self):
        self.map_label.setVisible(self.radio_map.isChecked())
        self.sensor_box.setVisible(self.radio_sensor.isChecked())
