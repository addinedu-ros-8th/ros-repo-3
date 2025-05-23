from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap
from viewer.theme import apply_theme
from shared_interfaces.msg import RobotInfo, AccessResult
import requests

class MonitorPanel(QWidget):
    def __init__(self):
        super().__init__()
        apply_theme(self)
        self._init_ui()

        # ROS2 퍼블리셔 생성
        self.roscar_info_publisher = self.create_publisher(RobotInfo, '/roscar/access', 10)


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
        roscar_manage_group = QGroupBox("Robot Management")
        roscar_manage_layout = QVBoxLayout()

        self.roscar_list = QListWidget()
        
        # 로봇 ID와 IP 입력 필드 추가
        self.roscar_id_input = QLineEdit(self)
        self.roscar_id_input.setPlaceholderText("Enter Robot ID")
        roscar_manage_layout.addWidget(self.roscar_id_input)

        self.roscar_ip_input = QLineEdit(self)
        self.roscar_ip_input.setPlaceholderText("Enter Robot IP Address")
        roscar_manage_layout.addWidget(self.roscar_ip_input)

        self.add_roscar_btn = QPushButton("Add Robot")
        self.remove_roscar_btn = QPushButton("Remove Robot")

        # 버튼 클릭 시 add_roscar 함수 호출
        self.add_roscar_btn.clicked.connect(self.add_roscar)

        roscar_manage_layout.addWidget(self.roscar_list)
        roscar_manage_layout.addWidget(self.add_roscar_btn)
        roscar_manage_layout.addWidget(self.remove_roscar_btn)
        roscar_manage_group.setLayout(roscar_manage_layout)

        top_layout.addWidget(roscar_manage_group)
        main_layout.addLayout(top_layout)

        # 2. 중간: Robot Status 테이블
        roscar_status_group = QGroupBox("Robot Status Overview")
        roscar_status_layout = QVBoxLayout()

        self.roscar_table = QTableWidget(0, 3)
        self.roscar_table.setHorizontalHeaderLabels(["Robot ID", "Battery", "Status"])
        self.roscar_table.horizontalHeader().setStretchLastSection(True)
        self.roscar_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

        roscar_status_layout.addWidget(self.roscar_table)
        roscar_status_group.setLayout(roscar_status_layout)
        main_layout.addWidget(roscar_status_group)

        # 3. 하단: Log Tabs (재고/로봇 이벤트/작업 로그)
        self.tab_widget = QTabWidget()

        self.stock_log_table = self._create_log_table(["Item", "Quantity", "Last Updated"])
        self.roscar_event_log_table = self._create_log_table(["Timestamp", "Robot", "Event"])
        self.task_log_table = self._create_log_table(["Timestamp", "Task ID", "Status"])

        self.tab_widget.addTab(self.stock_log_table, "Stock Log")
        self.tab_widget.addTab(self.roscar_event_log_table, "Robot Event Log")
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

    def update_roscar_list(self, roscar_name):
        # 로봇 리스트에 새로운 로봇 이름 추가
        self.roscar_list.addItem(roscar_name)

    def add_roscar(self):
        # 사용자가 입력한 로봇 ID와 IP 주소를 가져옵니다.
        roscar_id = self.roscar_id_input.text()
        roscar_ip = self.roscar_ip_input.text()

        if not roscar_id or not roscar_ip:
            self.show_message("Error", "Please provide both roscar ID and IP address.")
            return

        # 서버로 POST 요청
        msg = RobotInfo()
        msg.roscar_name = roscar_id
        msg.roscar_ip = roscar_ip
        self.roscar_info_publisher.publish(msg)
        
        # try:
        #     response = requests.post("http://192.168.0.168:5000/add_roscar", json=data)
        #     if response.status_code == 200:
        #         self.show_message("Success", "Robot added successfully!")
        #         self.update_roscar_list(roscar_id)  # Update roscar list in the GUI
        #     else:
        #         self.show_message("Failure", f"Failed to add roscar: {response.text}")
        # except requests.exceptions.RequestException as e:
        #     self.show_message("Error", f"Error: {e}")

    def show_message(self, title, message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Icon.Information)  # 수정된 부분
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec()
