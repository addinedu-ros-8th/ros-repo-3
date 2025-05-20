import os
import json
import yaml
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QTransform, QPainter, QColor
from viewer.theme import apply_theme
from shared_interfaces.msg import RoscarInfo, RoscarRegister
from shared_interfaces.srv import LogQuery
from geometry_msgs.msg import PoseStamped

class MonitorPanel(QWidget):
    def __init__(self):
        super().__init__()
        apply_theme(self)
        self._init_ui()

        rclpy.init(args=None)
        self.node = Node('monitor_panel')

        # Roscar Info publisher & register subscriber
        self.roscar_info_publisher = self.node.create_publisher(RoscarInfo, '/roscar/access', 10)
        self.roscar_register_subscriber = self.node.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.roscar_register_callback,
            10
        )

        # Main server pose subscriber
        self.pose_subscriber = self.node.create_subscription(
            PoseStamped,
            '/roscar/pose',
            self._pose_callback,
            10
        )

        # LogQuery client
        self.log_query_client = self.node.create_client(LogQuery, '/log/request/query')

        # Spin timer for ROS callbacks
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        self.ros_timer.start(50)

        self._load_all_logs()

        # Temporary test: draw a fake pose at (1.5, 2.0)m
        fake = PoseStamped()
        fake.pose.position.x = 36.0
        fake.pose.position.y = 1.5
        self._pose_callback(fake)

    def _init_ui(self):
        main_layout = QVBoxLayout()

        # Map display
        map_group = QGroupBox("Map View")
        map_layout = QVBoxLayout()

        # Determine paths relative to this file
        current_file = os.path.abspath(__file__)
        project_root = os.path.abspath(os.path.join(current_file, "../../../../"))
        yaml_path = os.path.join(
            project_root,
            "roscars/pinky_navigation/map/roscars_map.yaml"
        )

        image_path = None
        try:
            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                # Save map parameters
                self.resolution = map_data['resolution']
                self.origin_x, self.origin_y, _ = map_data['origin']
                image_file = map_data.get('image', 'roscars_map.pgm')
                image_path = os.path.join(os.path.dirname(yaml_path), image_file)
        except Exception as e:
            print(f"map.yaml parse failed: {e}")

        self.map_label = QLabel()
        if image_path and os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            if pixmap.isNull():
                print("Failed to load map image: QPixmap isNull")
            else:
                # Rotate 90° right
                transform = QTransform().rotate(-90)
                pixmap = pixmap.transformed(transform)
                # Scale to fit
                pixmap = pixmap.scaled(
                    800, 400,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation
                )
                self.map_label.setPixmap(pixmap)
                self.map_label.setFixedSize(pixmap.width(), pixmap.height())
                # Store for overlay
                self.base_pixmap = pixmap
        else:
            print(f"Map image not found: {image_path}")
            self.map_label.setFixedSize(800, 400)

        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setStyleSheet("background-color: #e7e3d4; border: 2px solid #a89f7d;")

        map_layout.addWidget(self.map_label)
        map_group.setLayout(map_layout)
        main_layout.addWidget(map_group)

        # Roscar Status Overview
        roscar_status_group = QGroupBox("Roscar Status Overview")
        roscar_status_layout = QVBoxLayout()
        self.roscar_table = QTableWidget(0, 3)
        self.roscar_table.setHorizontalHeaderLabels(["Roscar ID", "Battery", "Status"])
        self.roscar_table.horizontalHeader().setStretchLastSection(True)
        self.roscar_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        roscar_status_layout.addWidget(self.roscar_table)
        roscar_status_group.setLayout(roscar_status_layout)
        main_layout.addWidget(roscar_status_group)

        # Log tabs
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
        for qtype, headers in self.query_map.items():
            table = self._create_log_table(headers)
            self.tab_widget.addTab(table, qtype.replace('_',' ').title())
            self.log_tables[qtype] = table
        main_layout.addWidget(self.tab_widget)

        self.setLayout(main_layout)

    def _load_all_logs(self):
        for qtype, table in self.log_tables.items():
            self._query_and_display(qtype, table)

    def _query_and_display(self, query_type, table):
        if not self.log_query_client.wait_for_service(timeout_sec=2.0):
            return
        req = LogQuery.Request()
        req.query_type = query_type
        fut = self.log_query_client.call_async(req)
        def cb(fut):
            try:
                res = fut.result()
                data = json.loads(res.json_result)
                table.setRowCount(0)
                for row in data:
                    idx = table.rowCount()
                    table.insertRow(idx)
                    for col, val in enumerate(row.values()):
                        table.setItem(idx, col, QTableWidgetItem(str(val)))
            except Exception as e:
                self.show_message("Error", f"{query_type} load failed: {e}")
        fut.add_done_callback(cb)

    def _create_log_table(self, headers):
        tbl = QTableWidget(0, len(headers))
        tbl.setHorizontalHeaderLabels(headers)
        tbl.horizontalHeader().setStretchLastSection(True)
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        tbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        return tbl

    def update_roscar_list(self, namespace):
        self.roscar_table.insertRow(self.roscar_table.rowCount())
        self.roscar_table.setItem(self.roscar_table.rowCount()-1, 0, QTableWidgetItem(namespace))

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
            res = future.result()
            data = json.loads(res.json_result)
            self.task_log_table.setRowCount(0)
            for row in data:
                idx = self.task_log_table.rowCount()
                self.task_log_table.insertRow(idx)
                self.task_log_table.setItem(idx, 0, QTableWidgetItem(str(row.get("timestamp"))))
                self.task_log_table.setItem(idx, 1, QTableWidgetItem(str(row.get("delivery_id","-"))))
                self.task_log_table.setItem(idx, 2, QTableWidgetItem(str(row.get("event_type","-"))))
        except Exception as e:
            self.show_message("Error", f"Failed to parse logs: {e}")

    def roscar_register_callback(self, msg: RoscarRegister):
        self.update_roscar_table(msg.roscar_namespace)

    def show_message(self, title, text):
        dlg = QMessageBox()
        dlg.setIcon(QMessageBox.Icon.Information)
        dlg.setWindowTitle(title)
        dlg.setText(text)
        dlg.exec()

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

    # Callback for PoseStamped from main server
    def _pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        px = int((x - self.origin_x) / self.resolution)
        py = int(self.base_pixmap.height() - (y - self.origin_y) / self.resolution)
        overlay = QPixmap(self.base_pixmap)
        painter = QPainter(overlay)
        painter.setBrush(QColor(255, 0, 0, 100))
        painter.setPen(Qt.PenStyle.NoPen)
        r = 10
        painter.drawEllipse(px - r, py - r, 2*r, 2*r)
        painter.end()
        self.map_label.setPixmap(overlay)
