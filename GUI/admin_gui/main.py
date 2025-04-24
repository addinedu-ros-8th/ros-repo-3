import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from .dashboard_panel import MonitorPanel
from .dialogs import LogDialog, RobotDialog, RequestDialog
from .theme import apply_kaki_theme


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Admin Dashboard")
        self.setGeometry(150, 150, 1400, 900)

        apply_kaki_theme(self)

        # 중앙 위젯 설정
        central = QWidget()
        layout = QVBoxLayout()

        # 메인 대시보드 패널 구성
        self.monitor_panel = MonitorPanel()
        layout.addWidget(self.monitor_panel)

        # 테마 적용
        apply_kaki_theme(self.monitor_panel)

        # 각 버튼에 대응되는 다이얼로그 연결
        self.monitor_panel.btn_robots.clicked.connect(self.open_robot_dialog)
        self.monitor_panel.btn_requests.clicked.connect(self.open_request_dialog)
        self.monitor_panel.btn_logs.clicked.connect(self.open_log_dialog)

        central.setLayout(layout)
        self.setCentralWidget(central)

    def open_log_dialog(self):
        dialog = LogDialog()
        apply_kaki_theme(dialog)
        dialog.exec_()

    def open_robot_dialog(self):
        dialog = RobotDialog()
        apply_kaki_theme(dialog)
        dialog.exec_()

    def open_request_dialog(self):
        dialog = RequestDialog()
        apply_kaki_theme(dialog)
        dialog.exec_()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
