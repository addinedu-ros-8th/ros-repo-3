from PyQt6.QtWidgets import (
    QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QListWidget,
    QGroupBox, QTableWidget, QTableWidgetItem, QTabWidget, QHeaderView, QSizePolicy
)
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt
from viewer.theme import apply_theme

class MonitorPanel(QWidget):
    def __init__(self):
        super().__init__()
        apply_theme(self)
        self._init_ui()

    def _init_ui(self):
        main_layout = QVBoxLayout()

        # 1. 상단: Map + Robot 등록/삭제
        top_layout = QHBoxLayout()

        # Map 영역
        self.map_label = QLabel()
        self.map_label.setPixmap(QPixmap("/path/to/your/static_map.png"))  # <-- 이미지 경로 교체 필요
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setFixedSize(600, 400)
        self.map_label.setStyleSheet("background-color: #e7e3d4; border: 2px solid #a89f7d;")
        top_layout.addWidget(self.map_label)

        # Robot 관리 영역
        robot_manage_group = QGroupBox("Robot Management")
        robot_manage_layout = QVBoxLayout()

        self.robot_list = QListWidget()
        self.add_robot_btn = QPushButton("Add Robot")
        self.remove_robot_btn = QPushButton("Remove Robot")

        robot_manage_layout.addWidget(self.robot_list)
        robot_manage_layout.addWidget(self.add_robot_btn)
        robot_manage_layout.addWidget(self.remove_robot_btn)
        robot_manage_group.setLayout(robot_manage_layout)

        top_layout.addWidget(robot_manage_group)
        main_layout.addLayout(top_layout)

        # 2. 중간: Robot Status 테이블
        robot_status_group = QGroupBox("Robot Status Overview")
        robot_status_layout = QVBoxLayout()

        self.robot_table = QTableWidget(0, 3)
        self.robot_table.setHorizontalHeaderLabels(["Robot ID", "Battery", "Status"])
        self.robot_table.horizontalHeader().setStretchLastSection(True)
        self.robot_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

        robot_status_layout.addWidget(self.robot_table)
        robot_status_group.setLayout(robot_status_layout)
        main_layout.addWidget(robot_status_group)

        # 3. 하단: Log Tabs (재고/로봇 이벤트/작업 로그)
        self.tab_widget = QTabWidget()

        self.stock_log_table = self._create_log_table(["Item", "Quantity", "Last Updated"])
        self.robot_event_log_table = self._create_log_table(["Timestamp", "Robot", "Event"])
        self.task_log_table = self._create_log_table(["Timestamp", "Task ID", "Status"])

        self.tab_widget.addTab(self.stock_log_table, "Stock Log")
        self.tab_widget.addTab(self.robot_event_log_table, "Robot Event Log")
        self.tab_widget.addTab(self.task_log_table, "Task Log")

        main_layout.addWidget(self.tab_widget)

        self.setLayout(main_layout)

    def _create_log_table(self, headers):
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        return table

    # 추가 메소드 (로봇 리스트 업데이트, 테이블 데이터 추가 등 필요시 작성 가능)