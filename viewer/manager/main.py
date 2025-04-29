import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'viewer')))  # 경로 추가

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from viewer.theme import apply_theme  # theme.py 임포트
from .dashboard_panel import MonitorPanel
from .dialogs import LogDialog, RobotDialog, RequestDialog

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Manager Dashboard")
        self.setGeometry(150, 150, 1400, 900)

        apply_theme(self)  # 테마 적용

        central = QWidget()
        layout = QVBoxLayout()

        self.monitor_panel = MonitorPanel()
        layout.addWidget(self.monitor_panel)

        apply_theme(self.monitor_panel)  # 모니터 패널에도 테마 적용

        self.monitor_panel.btn_robots.clicked.connect(self.open_robot_dialog)
        self.monitor_panel.btn_requests.clicked.connect(self.open_request_dialog)
        self.monitor_panel.btn_logs.clicked.connect(self.open_log_dialog)

        central.setLayout(layout)
        self.setCentralWidget(central)

    def open_log_dialog(self):
        dialog = LogDialog()
        apply_theme(dialog)
        dialog.exec()

    def open_robot_dialog(self):
        dialog = RobotDialog()
        apply_theme(dialog)
        dialog.exec()

    def open_request_dialog(self):
        dialog = RequestDialog()
        apply_theme(dialog)
        dialog.exec()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
