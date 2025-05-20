import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from shared_interfaces.msg import RoscarRegister
from shared_interfaces.srv import LogQuery
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QGroupBox,
    QTableWidget, QTabWidget, QHeaderView, QTableWidgetItem,
    QSizePolicy, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap
from viewer.theme import apply_theme

class MonitorPanel(QWidget):

    def __init__(self):
        super().__init__()
        apply_theme(self)
        self.map_msg = None
        self.robot_pose = None

        self._init_ui()

        # ROS2 초기화 및 Node 생성
        rclpy.init(args=None)  # ROS2 초기화
        self.node = Node('monitor_panel')  # Node 생성

        # ROS2 퍼블리셔 생성 (Roscar 추가용)
        self.roscar_info_publisher = self.node.create_publisher(RoscarInfo, '/roscar/access', 10)

        # [추가] ROS2 구독자 생성 (/roscar/register 구독)
        self.roscar_register_subscriber = self.node.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.roscar_register_callback,
            10
        )

        self.log_query_client = self.node.create_client(LogQuery, '/log/request/query')

        # [추가] QTimer를 활용해 rclpy.spin_once 주기 호출
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        self.ros_timer.start(50)

        self._load_all_logs()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)

        # 1. 상단: Map만 표시
        self.map_label = QLabel()
        self.map_label.setPixmap(QPixmap("/path/to/your/static_map.png"))  # 이미지 경로 수정 필요
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setFixedSize(1200, 400)
        self.map_label.setStyleSheet("background-color: #e7e3d4; border: 2px solid #a89f7d;")
        main_layout.addWidget(self.map_label)

        # 2. 중간: Roscar Status 테이블
        roscar_status_group = QGroupBox("Roscar Status Overview")
        roscar_status_layout = QVBoxLayout()

        self.roscar_table = QTableWidget(0, 3)
        self.roscar_table.setHorizontalHeaderLabels(
            ["Roscar ID", "Battery", "Status"]
        )
        self.roscar_table.horizontalHeader().setStretchLastSection(True)
        self.roscar_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        status_layout.addWidget(self.roscar_table)
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # 3. 하단: Log Tabs (모든 쿼리 포함)
        self.query_map = {
            "delivery": ["delivery_id","event","timestamp","user","roscar"],
            "task":     ["task_id",    "status", "timestamp","shoes","location"],
            "roscar":   ["event_id",   "roscar", "type",     "status","timestamp"],
            "precision_stop": ["roscar","success","deviation","timestamp"],
            "trajectory":     ["roscar","task","pos_x","pos_y","velocity"],
            "driving_event":  ["roscar","event","timestamp"],
            "sensor_fusion":  ["roscar","timestamp","lidar","imu","ultrasonic"],
            "control_command":["roscar","linear_vel","angular_vel","timestamp"],
            "filesystem":     ["roscar","file_path","timestamp"],
            "rack_sensor":    ["roscar","rack_id","status","x","y","z","timestamp"]
        }
        self.tab_widget = QTabWidget()
        self.log_tables = {}

        for query_type, headers in self.query_map.items():
            table = self._create_log_table(headers)
            self.tab_widget.addTab(table, query_type.replace("_", " ").title())
            self.log_tables[query_type] = table

        main_layout.addWidget(self.tab_widget)

        self.setLayout(main_layout)

    def _init_rclpy(self):
        rclpy.init(args=None)
        self.node = Node('monitor_panel')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # GUI용 토픽 구독
        self.node.create_subscription(
            OccupancyGrid, '/monitor/map', self._map_cb, qos
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/monitor/pose', self._pose_cb, qos
        )
        self.node.create_subscription(
            RoscarRegister, '/monitor/register', self._reg_cb, qos
        )
        # LogQuery 서비스 클라이언트
        self.log_client = self.node.create_client(
            LogQuery, '/log/request/query'
        )

        # spin_once 주기
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.node, timeout_sec=0.01)
        )
        self.ros_timer.start(10)

        # 맵 렌더링 주기
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_map_display)
        self.update_timer.start(50)

    # ---------------- 콜백 ----------------
    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg.pose.pose

    def _reg_cb(self, msg: RoscarRegister):
        self._update_roscar_table(
            msg.roscar_namespace,
            f"{msg.battery_percentage}%",
            msg.operational_status
        )

    # --------------- 로그 로딩 ---------------
    def _load_all_logs(self):
        for query_type, table in self.log_tables.items():
            self._query_and_display(query_type, table)


    def _query_and_display(self, query_type, table):
        if not self.log_query_client.wait_for_service(timeout_sec=2.0):
            return

        request = LogQuery.Request()
        request.query_type = query_type

        future = self.log_query_client.call_async(request)

        def callback(fut):
            try:
                result = fut.result()
                data = json.loads(result.json_result)
                table.setRowCount(0)
                for row in data:
                    row_index = table.rowCount()
                    table.insertRow(row_index)
                    for col, key in enumerate(row.values()):
                        table.setItem(row_index, col, QTableWidgetItem(str(key)))
            except Exception as e:
                self.show_message("Error", f"{query_type} 로딩 실패: {e}")

        future.add_done_callback(callback)


    def _create_log_table(self, headers):
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        return table

    def update_roscar_list(self, roscar_namespace):
        # 로봇 리스트에 새로운 로봇 이름 추가
        self.roscar_list.addItem(roscar_namespace)

    def add_roscar(self):
        # 사용자가 입력한 로봇 ID와 IP 주소를 가져옵니다.
        roscar_id = self.roscar_id_input.text()
        roscar_ip = self.roscar_ip_input.text()

        if not roscar_id or not roscar_ip:
            self.show_message("Error", "Please provide both roscar ID and IP address.")
            return

        # ROS2 메시지 전송
        msg = RoscarInfo()
        msg.roscar_namespace = roscar_id
        msg.roscar_ip = roscar_ip
        self.roscar_info_publisher.publish(msg)

        # 추가된 로봇 리스트에 표시
        self.update_roscar_list(roscar_id)

        self.show_message("Success", f"Roscar {roscar_id} added successfully!")

    def query_logs(self):
        if not self.log_query_client.wait_for_service(timeout_sec=2.0):
            self.show_message("Error", "LogQuery Service is not available.")
            return

        request = LogQuery.Request()
        request.query_type = "delivery"  # 또는 task, roscar, 등등

        future = self.log_query_client.call_async(request)
        future.add_done_callback(self.handle_log_response)

    def handle_log_response(self, future):
        try:
            response = future.result()
            data = json.loads(response.json_result)

            # 기존 테이블 초기화
            self.task_log_table.setRowCount(0)

            for row in data:
                row_index = self.task_log_table.rowCount()
                self.task_log_table.insertRow(row_index)
                self.task_log_table.setItem(row_index, 0, QTableWidgetItem(str(row.get("timestamp"))))
                self.task_log_table.setItem(row_index, 1, QTableWidgetItem(str(row.get("delivery_id", "-"))))
                self.task_log_table.setItem(row_index, 2, QTableWidgetItem(str(row.get("event_type", "-"))))
        except Exception as e:
            self.show_message("Error", f"Failed to parse response: {e}")


    def show_message(self, title, message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Icon.Information)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec()

    def closeEvent(self, event):
        # ROS2 노드 종료 처리
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

    # [추가] RoscarRegister 메시지를 수신하면 테이블에 반영하는 콜백
    def roscar_register_callback(self, msg):
        roscar_id = msg.roscar_namespace
        battery = f"{msg.battery_percentage}%"
        status = msg.operational_status
        self.update_roscar_table(roscar_id, battery, status)

    # [추가] Roscar Status Overview 테이블 갱신 함수
    def update_roscar_table(self, roscar_id, battery, status):
        table = self.roscar_table
        for row in range(table.rowCount()):
            item = table.item(row, 0)
            if item and item.text() == roscar_id:
                table.setItem(row, 1, QTableWidgetItem(battery))
                table.setItem(row, 2, QTableWidgetItem(status))
                return

        row_position = table.rowCount()
        table.insertRow(row_position)
        table.setItem(row_position, 0, QTableWidgetItem(roscar_id))
        table.setItem(row_position, 1, QTableWidgetItem(battery))
        table.setItem(row_position, 2, QTableWidgetItem(status))

    # 콜백 메서드 구현

    def _pose_callback(self, msg: PoseStamped):
        # 1) 월드 좌표 추출
        x = msg.pose.position.x
        y = msg.pose.position.y

        # 2) 픽셀 좌표로 변환
        px = int((x - self.origin_x) / self.resolution)
        py = int(self.base_pixmap.height() - (y - self.origin_y) / self.resolution)

        # 3) 마커 그리기
        overlay = QPixmap(self.base_pixmap)
        painter = QPainter(overlay)
        painter.setBrush(QColor(255, 0, 0, 100))
        painter.setPen(Qt.PenCapStyle.Nopen)
        r = 10
        painter.drawEllipse(px - r, py - r, 2*r, 2*r)
        painter.end()

        # 4) Qlabel 갱싱
        self.map_label.setPixmap(overlay)