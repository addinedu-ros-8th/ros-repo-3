import os
import json
import yaml
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QTransform
from viewer.theme import apply_theme
from shared_interfaces.msg import RoscarInfo, RoscarRegister
from shared_interfaces.srv import LogQuery
from geometry_msgs.msg import PoseStamped
from PyQt6.QtGui import QPainter, QColor



class MonitorPanel(QWidget):

    def __init__(self):
        super().__init__()
        apply_theme(self)
        self._init_ui()

        rclpy.init(args=None)
        self.node = Node('monitor_panel')

        self.roscar_info_publisher = self.node.create_publisher(RoscarInfo, '/roscar/access', 10)
        self.roscar_register_subscriber = self.node.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.roscar_register_callback,
            10
        )
        # 구독자 추가
        self.pose_subscriber = self.node.create_subscription(
            PoseStamped,
            "/roscar/pose",         # 메인 서버가 퍼블리셔하는 토픽
            self._pose_callback,     # 아래에 정의할 콜밸
            10
        )
    


        self.log_query_client = self.node.create_client(LogQuery, '/log/request/query')

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        self.ros_timer.start(50)

        self._load_all_logs()

    def _init_ui(self):
        main_layout = QVBoxLayout()

        # 지도 표시 영역
        map_group = QGroupBox("Map View")
        map_layout = QVBoxLayout()
        # map_layout.setContentsMargins(0, 0, 0, 0)    # 레이아웃 여백 제거
        # map_layout.setSpacing(0)                     # 위젯 간 간격 제거

        # 프로젝트 루트 기준 map.yaml 경로
        current_file = os.path.abspath(__file__)
        project_root = os.path.abspath(os.path.join(current_file, "../../../../"))
        yaml_path = os.path.join(
            project_root,
            "roscars/pinky_violet/pinky_navigation/map/roscars_map.yaml"
        )

        image_path = None
        try:
            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                # map_data 파싱 직후에 좌표 변환용 변수 저장
                self.resolution = map_data['resolution']
                self.origin_x, self.origin_y, _ = map_data['origin']

                image_file = map_data.get("image", "map.pgm")
                image_path = os.path.join(os.path.dirname(yaml_path), image_file)
        except Exception as e:
            print(f"map.yaml 파싱 실패: {e}")

        self.map_label = QLabel()
        if image_path and os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            if pixmap.isNull():
                print("이미지 로딩 실패: QPixmap isNull")
            else:
                # 우측으로 90도 회전
                transform = QTransform().rotate(-90)
                pixmap = pixmap.transformed(transform)
                # 크기 조정 (최대 너비 1200, 최대 높이 600)
                pixmap = pixmap.scaled(800, 400, 
                                       Qt.AspectRatioMode.KeepAspectRatio,
                                       #Qt.TransformationMode.FastTransformation  #픽셀 에지를 정확히 유지하고 싶으면
                                       Qt.TransformationMode.SmoothTransformation #선명한 블록 형태 계단 현상이 싫으면
                                       )
                self.map_label.setPixmap(pixmap)
                self.map_label.setFixedSize(pixmap.width(), pixmap.height())
        else:
            print(f"이미지 파일이 존재하지 않음: {image_path}")
            self.map_label.setFixedSize(800, 400)

        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setStyleSheet("background-color: #e7e3d4; border: 2px solid #a89f7d;")

        map_layout.addWidget(self.map_label)
        map_group.setLayout(map_layout)
        main_layout.addWidget(map_group)

        # Roscar 상태 테이블
        roscar_status_group = QGroupBox("Roscar Status Overview")
        roscar_status_layout = QVBoxLayout()

        self.roscar_table = QTableWidget(0, 3)
        self.roscar_table.setHorizontalHeaderLabels(["Roscar ID", "Battery", "Status"])
        self.roscar_table.horizontalHeader().setStretchLastSection(True)
        self.roscar_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

        roscar_status_layout.addWidget(self.roscar_table)
        roscar_status_group.setLayout(roscar_status_layout)
        main_layout.addWidget(roscar_status_group)

        # 로그 탭
        self.query_map = {
            "delivery": ["Delivery ID", "Event", "Timestamp", "User", "Roscar"],
            "task": ["Task ID", "Status", "Timestamp", "Shoes", "Location"],
            "roscar": ["Event ID", "Roscar", "Type", "Status", "Timestamp"],
            "precision_stop": ["Roscar", "Success", "Deviation", "Timestamp"],
            "trajectory": ["Roscar", "Task", "PosX", "PosY", "Velocity"],
            "driving_event": ["Roscar", "Event", "Timestamp"],
            "sensor_fusion": ["Roscar", "Timestamp", "LiDAR", "IMU", "Ultrasonic"],
            "control_command": ["Roscar", "Linear Vel", "Angular Vel", "Timestamp"],
            "filesystem": ["Roscar", "File Path", "Timestamp"],
            "rack_sensor": ["Roscar", "Rack ID", "Status", "X", "Y", "Z", "Timestamp"]
        }

        self.tab_widget = QTabWidget()
        self.log_tables = {}
        for query_type, headers in self.query_map.items():
            table = self._create_log_table(headers)
            self.tab_widget.addTab(table, query_type.replace("_", " ").title())
            self.log_tables[query_type] = table

        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

    def _load_all_logs(self):
        for qtype, table in self.log_tables.items():
            self._query_and_display(qtype, table)

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
                    idx = table.rowCount()
                    table.insertRow(idx)
                    for col, val in enumerate(row.values()):
                        table.setItem(idx, col, QTableWidgetItem(str(val)))
            except Exception as e:
                self.show_message("Error", f"{query_type} 로딩 실패: {e}")
        future.add_done_callback(callback)

    def _create_log_table(self, headers):
        tbl = QTableWidget(0, len(headers))
        tbl.setHorizontalHeaderLabels(headers)
        tbl.horizontalHeader().setStretchLastSection(True)
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        tbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        return tbl

    def update_roscar_list(self, roscar_namespace):
        self.roscar_list.addItem(roscar_namespace)

    def add_roscar(self):
        rid = self.roscar_id_input.text()
        rip = self.roscar_ip_input.text()
        if not rid or not rip:
            self.show_message("Error", "Please provide both roscar ID and IP address.")
            return
        msg = RoscarInfo()
        msg.roscar_namespace = rid
        msg.roscar_ip = rip
        self.roscar_info_publisher.publish(msg)
        self.update_roscar_list(rid)
        self.show_message("Success", f"Roscar {rid} added successfully.")

    def query_logs(self):
        if not self.log_query_client.wait_for_service(timeout_sec=2.0):
            self.show_message("Error", "LogQuery Service is not available.")
            return
        req = LogQuery.Request()
        req.query_type = "delivery"
        fut = self.log_query_client.call_async(req)
        fut.add_done_callback(self.handle_log_response)

    def handle_log_response(self, future):
        try:
            resp = future.result()
            data = json.loads(resp.json_result)
            self.task_log_table.setRowCount(0)
            for row in data:
                idx = self.task_log_table.rowCount()
                self.task_log_table.insertRow(idx)
                self.task_log_table.setItem(idx, 0, QTableWidgetItem(str(row.get("timestamp"))))
                self.task_log_table.setItem(idx, 1, QTableWidgetItem(str(row.get("delivery_id", "-"))))
                self.task_log_table.setItem(idx, 2, QTableWidgetItem(str(row.get("event_type", "-"))))
        except Exception as e:
            self.show_message("Error", f"Failed to parse response: {e}")

    def show_message(self, title, message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Icon.Information)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec()

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

    def roscar_register_callback(self, msg):
        rid = msg.roscar_namespace
        battery = f"{msg.battery_percentage}%"
        status = msg.operational_status
        self.update_roscar_table(rid, battery, status)

    def update_roscar_table(self, roscar_id, battery, status):
        tbl = self.roscar_table
        for r in range(tbl.rowCount()):
            item = tbl.item(r, 0)
            if item and item.text() == roscar_id:
                tbl.setItem(r, 1, QTableWidgetItem(battery))
                tbl.setItem(r, 2, QTableWidgetItem(status))
                return
        pos = tbl.rowCount()
        tbl.insertRow(pos)
        tbl.setItem(pos, 0, QTableWidgetItem(roscar_id))
        tbl.setItem(pos, 1, QTableWidgetItem(battery))
        tbl.setItem(pos, 2, QTableWidgetItem(status))

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