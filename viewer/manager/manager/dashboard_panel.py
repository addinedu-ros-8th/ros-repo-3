import sys, json
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
from PyQt6.QtGui import QPixmap, QImage, QPainter, QColor
from viewer.theme import apply_theme

class MonitorPanel(QWidget):
    def __init__(self):
        super().__init__()
        apply_theme(self)
        self.map_msg = None
        self.robot_pose = None

        self._init_ui()
        self._init_rclpy()
        self._load_all_logs()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)

        # 1) Map 표시 영역
        self.map_label = QLabel()
        self.map_label.setFixedSize(1200, 400)
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setStyleSheet(
            "background-color: #e7e3d4; border: 2px solid #a89f7d;"
        )
        main_layout.addWidget(self.map_label)

        # 2) Roscar Status 테이블
        status_group = QGroupBox("Roscar Status Overview")
        status_layout = QVBoxLayout()
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

        # 3) Log 탭
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
        for qtype, headers in self.query_map.items():
            tbl = self._create_log_table(headers)
            self.tab_widget.addTab(tbl, qtype.replace("_"," ").title())
            self.log_tables[qtype] = tbl
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
        for qtype, tbl in self.log_tables.items():
            self._query_and_display(qtype, tbl)

    def _query_and_display(self, qtype, table):
        if not self.log_client.wait_for_service(timeout_sec=2.0):
            return
        req = LogQuery.Request()
        req.query_type = qtype
        fut = self.log_client.call_async(req)
        fut.add_done_callback(
            lambda f, qt=qtype, tb=table: self._on_log_response(f, qt, tb)
        )

    def _on_log_response(self, future, qtype, table):
        try:
            res = future.result()
            raw = res.json_result
            data = json.loads(raw)

            # 단일 dict이면 리스트로 감싸고, 아닌 경우 그대로 사용
            if isinstance(data, dict):
                data = [data]
            elif not isinstance(data, list):
                raise ValueError(f"예상 리스트 형태, 받음: {type(data)}")

            headers = self.query_map[qtype]
            table.setRowCount(0)

            for row in data:
                r = table.rowCount()
                table.insertRow(r)
                for c, key in enumerate(headers):
                    val = row.get(key, "")
                    table.setItem(r, c, QTableWidgetItem(str(val)))

        except Exception as e:
            self._show_message("Error", f"{qtype} 로딩 실패: {e}")

    # ------------- 테이블 생성 -------------
    def _create_log_table(self, headers):
        tbl = QTableWidget(0, len(headers))
        # 컬럼 이름을 보기 편하게 title case 로 표시
        tbl.setHorizontalHeaderLabels([h.replace("_"," ").title() for h in headers])
        tbl.horizontalHeader().setStretchLastSection(True)
        tbl.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        tbl.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        return tbl

    # ------- Roscar 상태 테이블 갱신 -------
    def _update_roscar_table(self, rid, battery, status):
        table = self.roscar_table
        for row in range(table.rowCount()):
            if table.item(row, 0).text() == rid:
                table.setItem(row, 1, QTableWidgetItem(battery))
                table.setItem(row, 2, QTableWidgetItem(status))
                return
        r = table.rowCount()
        table.insertRow(r)
        table.setItem(r, 0, QTableWidgetItem(rid))
        table.setItem(r, 1, QTableWidgetItem(battery))
        table.setItem(r, 2, QTableWidgetItem(status))

    # -------- 맵 + 로봇 위치 렌더링 --------
    def _update_map_display(self):
        if not self.map_msg:
            return
        info = self.map_msg.info
        w, h = info.width, info.height
        data = self.map_msg.data

        img = QImage(w, h, QImage.Format.Format_Grayscale8)
        for yy in range(h):
            for xx in range(w):
                v = data[xx + yy * w]
                c = 255 - v if v >= 0 else 127
                img.setPixel(xx, h - yy - 1, QColor(c, c, c).rgb())

        pix = QPixmap.fromImage(img).scaled(
            self.map_label.width(),
            self.map_label.height(),
            Qt.AspectRatioMode.KeepAspectRatio
        )

        if self.robot_pose:
            ox = info.origin.position.x
            oy = info.origin.position.y
            res = info.resolution
            px = int((self.robot_pose.position.x - ox) / res)
            py = int((self.robot_pose.position.y - oy) / res)
            py = h - py - 1
            sx = pix.width() / w
            sy = pix.height() / h

            painter = QPainter(pix)
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(int(px * sx) - 5, int(py * sy) - 5, 10, 10)
            painter.end()

        self.map_label.setPixmap(pix)

    # ---------------- 유틸 ----------------
    def _show_message(self, title, msg):
        dlg = QMessageBox(self)
        dlg.setIcon(QMessageBox.Icon.Information)
        dlg.setWindowTitle(title)
        dlg.setText(msg)
        dlg.exec()

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MonitorPanel()
    win.show()
    sys.exit(app.exec())
