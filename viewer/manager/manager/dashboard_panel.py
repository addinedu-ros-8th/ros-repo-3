# dashboard_panel.py
import os, yaml, json, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QGroupBox,
    QTableWidget, QTabWidget, QHeaderView,
    QSizePolicy, QMessageBox, QTableWidgetItem
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QTransform, QPainter, QColor, QImage

from geometry_msgs.msg import PoseStamped
from shared_interfaces.msg import RoscarRegister, RoscarInfo
from shared_interfaces.srv import LogQuery
from viewer.theme import apply_theme

class MonitorPanel(QWidget):
    def __init__(self):
        super().__init__()
        apply_theme(self)

        self.base_pixmap = None
        self.map_width   = 0
        self.map_height  = 0
        self.origin_x    = None
        self.origin_y    = None
        self.resolution  = None
        self.robot_pose  = None

        self._init_ui()
        self._load_map_yaml_and_pgm()
        self._init_ros()
        self._redraw_map()
        self._load_all_logs()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        self.map_label = QLabel()
        self.map_label.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed
        )
        self.map_label.setMaximumHeight(400)
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setStyleSheet(
            "background-color: #e7e3d4; border: 2px solid #a89f7d;"
        )
        layout.addWidget(self.map_label)

        grp = QGroupBox("Roscar Status Overview")
        v = QVBoxLayout()
        self.roscar_table = QTableWidget(0, 3)
        self.roscar_table.setHorizontalHeaderLabels(
            ["Roscar ID", "Battery", "Status"]
        )
        self.roscar_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        v.addWidget(self.roscar_table)
        grp.setLayout(v)
        layout.addWidget(grp)

        self.query_map = {
            "delivery":       ["delivery_id","event","timestamp","user","roscar"],
            "task":           ["task_id","status","timestamp","shoes","location"],
            "roscar":         ["event_id","roscar","type","status","timestamp"],
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
        for key, headers in self.query_map.items():
            tbl = QTableWidget(0, len(headers))
            tbl.setHorizontalHeaderLabels(headers)
            tbl.horizontalHeader().setSectionResizeMode(
                QHeaderView.ResizeMode.Stretch
            )
            self.tab_widget.addTab(tbl, key.replace("_"," ").title())
            self.log_tables[key] = tbl
        layout.addWidget(self.tab_widget)

        self.setLayout(layout)

    def _load_map_yaml_and_pgm(self):
        pgm_path  = "/home/sang/dev_ws/git_ws/ros-repo-3/roscars/pinky_navigation/map/roscars_map.pgm"
        yaml_path = pgm_path.replace(".pgm", ".yaml")
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                cfg = yaml.safe_load(f)
                self.resolution = cfg.get('resolution', 1.0)
                ox, oy, _    = cfg.get('origin', [0.0,0.0,0.0])
                self.origin_x, self.origin_y = ox, oy

        if os.path.exists(pgm_path):
            pix = QPixmap(pgm_path)
            if not pix.isNull():
                # 왼쪽으로 90도 회전
                rot = pix.transformed(QTransform().rotate(-90))
                self.base_pixmap = rot
                self.map_width   = rot.width()
                self.map_height  = rot.height()
            else:
                print(f"PGM 로드 실패: {pgm_path}")
        else:
            print(f"PGM 파일 없음: {pgm_path}")

    def _init_ros(self):
        rclpy.init(args=None)
        self.node = Node('monitor_panel')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.node.create_subscription(
            PoseStamped, '/main_server/robot_pose',
            self._pose_cb, qos
        )
        self.node.create_subscription(
            RoscarRegister, '/roscar/register',
            self.roscar_register_callback, 10
        )
        self.roscar_info_publisher = self.node.create_publisher(
            RoscarInfo, '/roscar/access', 10
        )
        self.log_query_client = self.node.create_client(
            LogQuery, '/log/request/query'
        )

        timer = QTimer(self)
        timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        timer.start(50)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._redraw_map()

    def _redraw_map(self):
        if self.base_pixmap is None:
            return

        scaled = self.base_pixmap.scaled(
            self.map_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        if (self.robot_pose is not None
            and self.origin_x is not None
            and self.origin_y is not None
            and self.resolution is not None):

            raw_x = (self.robot_pose.position.x - self.origin_x) / self.resolution
            raw_y = (self.robot_pose.position.y - self.origin_y) / self.resolution

            px = int(raw_x * (scaled.width()  / self.map_width))
            py = int((self.map_height - raw_y) * (scaled.height() / self.map_height))

            print(f"raw_x={raw_x:.1f}, raw_y={raw_y:.1f}, px={px}, py={py}")

            if 0 <= px < scaled.width() and 0 <= py < scaled.height():
                painter = QPainter(scaled)
                painter.setBrush(QColor(255, 0, 0, 180))
                r = 6
                painter.drawEllipse(px-r, py-r, 2*r, 2*r)
                painter.end()

        self.map_label.setPixmap(scaled)

    def _pose_cb(self, msg: PoseStamped):
        self.robot_pose = msg.pose
        self._redraw_map()

    def _load_all_logs(self):
        for qt, tbl in self.log_tables.items():
            if not self.log_query_client.wait_for_service(timeout_sec=2.0):
                continue
            req = LogQuery.Request()
            req.query_type = qt
            fut = self.log_query_client.call_async(req)
            fut.add_done_callback(lambda f, t=tbl: self._on_log_response(f, t))

    def _on_log_response(self, future, tbl):
        try:
            res  = future.result()
            data = json.loads(res.json_result)
            tbl.setRowCount(0)
            for row in data:
                i = tbl.rowCount()
                tbl.insertRow(i)
                for c, v in enumerate(row.values()):
                    tbl.setItem(i, c, QTableWidgetItem(str(v)))
        except Exception as e:
            self.show_message("Error", f"로그 로딩 실패: {e}")

    def roscar_register_callback(self, msg: RoscarRegister):
        self.update_roscar_table(
            msg.roscar_namespace,
            f"{msg.battery_percentage}%",
            msg.operational_status
        )

    def update_roscar_table(self, ns, battery, status):
        tbl = self.roscar_table
        for r in range(tbl.rowCount()):
            if tbl.item(r,0).text() == ns:
                tbl.setItem(r,1,QTableWidgetItem(battery))
                tbl.setItem(r,2,QTableWidgetItem(status))
                return
        i = tbl.rowCount()
        tbl.insertRow(i)
        tbl.setItem(i,0,QTableWidgetItem(ns))
        tbl.setItem(i,1,QTableWidgetItem(battery))
        tbl.setItem(i,2,QTableWidgetItem(status))

    def show_message(self, title, msg):
        box = QMessageBox(self)
        box.setWindowTitle(title)
        box.setText(msg)
        box.exec()

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()
