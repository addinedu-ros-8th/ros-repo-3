import os
import yaml
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QGroupBox,
    QTableWidget, QTabWidget, QHeaderView,
    QSizePolicy, QMessageBox, QTableWidgetItem
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap, QTransform, QPainter, QColor
from gui.shared.theme import apply_theme

class MonitorPanel(QWidget):
    def __init__(self, ros_interface):
        super().__init__()
        apply_theme(self)
        self.ros_interface = ros_interface

        # 지도 표시용 변수 초기화
        self.base_pixmap = None
        self.orig_w = self.orig_h = 0
        self.map_width = self.map_height = 0
        self.origin_x = self.origin_y = self.resolution = None
        self.roscar_pose = None

        # UI 초기화
        self._init_ui()
        self._load_map_yaml_and_pgm()
        self._connect_ros_signals()
        self._redraw_map()

        # 초기 로그 요청
        for query_type in self.query_map.keys():
            self.ros_interface.request_log(query_type)
        # 실시간 RosCars 상태 첫 요청
        self.ros_interface.request_status()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # 맵 레이블
        self.map_label = QLabel()
        self.map_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.map_label.setMaximumHeight(400)
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setStyleSheet("background-color: #ffffff; border: 2px solid #ffffff;")
        layout.addWidget(self.map_label)

        # Roscar 상태 테이블
        grp = QGroupBox("Roscar Status Overview")
        v = QVBoxLayout()
        self.roscar_table = QTableWidget(0, 3)
        self.roscar_table.setHorizontalHeaderLabels(["Roscar SSID", "Battery", "Drive_Status"])
        self.roscar_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        v.addWidget(self.roscar_table)
        grp.setLayout(v)
        layout.addWidget(grp)

        # 로그 탭
        self.query_map = {
            "delivery":       ["delivery_id", "event", "timestamp", "user", "roscar"],
            "task":           ["task_id", "status", "timestamp", "shoes", "location"],
            "roscar":         ["event_id", "roscar", "type", "status", "timestamp"],
            "precision_stop": ["roscar", "success", "deviation", "timestamp"],
            "trajectory":     ["roscar", "task", "pos_x", "pos_y", "velocity"],
            "driving_event":  ["roscar", "event", "timestamp"],
            "sensor_fusion":  ["roscar", "timestamp", "lidar", "imu", "ultrasonic"],
            "control_command": ["roscar", "linear_vel", "angular_vel", "timestamp"],
            "filesystem":     ["roscar", "file_path", "timestamp"],
            "rack_sensor":    ["roscar", "rack_id", "status", "x", "y", "z", "timestamp"]
        }
        self.tab_widget = QTabWidget()
        self.log_tables = {}
        for key, headers in self.query_map.items():
            tbl = QTableWidget(0, len(headers))
            tbl.setHorizontalHeaderLabels(headers)
            tbl.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
            self.tab_widget.addTab(tbl, key.replace("_", " ").title())
            self.log_tables[key] = tbl
        layout.addWidget(self.tab_widget)

        self.setLayout(layout)

    def _load_map_yaml_and_pgm(self):
        pgm_path = "roscars/pinky_navigation/map/roscars_map.pgm"
        yaml_path = pgm_path.replace(".pgm", ".yaml")

        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                cfg = yaml.safe_load(f)
                self.resolution = cfg.get('resolution', 1.0)
                ox, oy, _ = cfg.get('origin', [0.0, 0.0, 0.0])
                self.origin_x, self.origin_y = ox, oy

        if os.path.exists(pgm_path):
            orig = QPixmap(pgm_path)
            if not orig.isNull():
                self.orig_w, self.orig_h = orig.width(), orig.height()
                rot = orig.transformed(QTransform().rotate(-90))
                self.base_pixmap = rot
                self.map_width = rot.width()
                self.map_height = rot.height()
            else:
                print(f"[MAP] PGM 로드 실패: {pgm_path}")
        else:
            print(f"[MAP] PGM 파일 없음: {pgm_path}")

    def _redraw_map(self):
        if self.base_pixmap is None:
            return

        scaled = self.base_pixmap.scaled(
            self.map_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        if (
            self.roscar_pose is not None and
            self.origin_x is not None and
            self.origin_y is not None and
            self.resolution is not None
        ):
            u0 = (self.roscar_pose.position.x - self.origin_x) / self.resolution
            v0 = (self.roscar_pose.position.y - self.origin_y) / self.resolution
            v_top = self.orig_h - 1 - v0

            u_rot = v_top
            v_rot = self.orig_w - 1 - u0

            sx = scaled.width() / self.map_width
            sy = scaled.height() / self.map_height

            px = int(u_rot * sx)
            py = int(v_rot * sy)

            if 0 <= px < scaled.width() and 0 <= py < scaled.height():
                painter = QPainter(scaled)
                painter.setBrush(QColor(255, 0, 0, 180))
                r = 20
                painter.drawEllipse(px - r, py - r, 2 * r, 2 * r)
                painter.end()

        self.map_label.setPixmap(scaled)

    def _connect_ros_signals(self):
        self.ros_interface.pose_received.connect(self._on_pose_received)
        self.ros_interface.roscar_registered.connect(self._on_roscar_registered)
        self.ros_interface.log_query_response.connect(self._on_log_query_response)
        self.ros_interface.roscar_status_response.connect(self._on_roscar_status_response)

    def _on_pose_received(self, msg):
        self.roscar_pose = msg.pose
        self._redraw_map()

    def _on_roscar_registered(self, msg):
        self.update_roscar_table(
            msg.roscar_namespace,
            f"{msg.battery_percentage}%",
            msg.operational_status
        )

    def _on_log_query_response(self, query_type, data):
        tbl = self.log_tables.get(query_type)
        if not tbl:
            return
        tbl.setRowCount(0)
        for row in data:
            i = tbl.rowCount()
            tbl.insertRow(i)
            for c, v in enumerate(row.values()):
                tbl.setItem(i, c, QTableWidgetItem(str(v)))

    def _on_roscar_status_response(self, data: list):
        print(f"[MonitorPanel] Received RosCars data: {data}")  # 디버그 로그
        print(f"[MonitorPanel] _on_roscar_status_response() data={data}")
        tbl = self.roscar_table
        tbl.setRowCount(0)
        for row in data:
            i = tbl.rowCount()
            tbl.insertRow(i)
            tbl.setItem(i, 0, QTableWidgetItem(row['roscar_namespace']))
            tbl.setItem(i, 1, QTableWidgetItem(f"{row['battery_percentage']}%"))
            tbl.setItem(i, 2, QTableWidgetItem(row['operational_status']))

    def update_roscar_table(self, ns, battery, status):
        tbl = self.roscar_table
        for r in range(tbl.rowCount()):
            if tbl.item(r, 0).text() == ns:
                tbl.setItem(r, 1, QTableWidgetItem(battery))
                tbl.setItem(r, 2, QTableWidgetItem(status))
                return
        i = tbl.rowCount()
        tbl.insertRow(i)
        tbl.setItem(i, 0, QTableWidgetItem(ns))
        tbl.setItem(i, 1, QTableWidgetItem(battery))
        tbl.setItem(i, 2, QTableWidgetItem(status))

    def show_message(self, title, msg):
        box = QMessageBox(self)
        box.setWindowTitle(title)
        box.setText(msg)
        box.exec()

    def closeEvent(self, event):
        self.ros_interface.shutdown()
        event.accept()