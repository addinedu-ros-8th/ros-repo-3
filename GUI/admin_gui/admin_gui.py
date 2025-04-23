# admin_gui.py
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from monitor_panel import MonitorPanel
from task_control_panel import LogDialog, RobotDialog, RequestDialog


def apply_kaki_theme(widget):
    widget.setStyleSheet("""
        QMainWindow {
            background-color: #f4f2ec;
        }
        QLabel#titleLabel {
            font-size: 22px;
            font-weight: bold;
            color: #3e3e3e;
        }
        QGroupBox {
            border: 1px solid #c2bca2;
            border-radius: 8px;
            margin-top: 10px;
            background-color: #fffdf6;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px 0 3px;
            font-weight: bold;
            color: #5b5a4e;
        }
        QPushButton {
            background-color: #708238;
            color: white;
            border-radius: 6px;
            padding: 8px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #5e6f2a;
        }
        QRadioButton {
            font-size: 14px;
            color: #3e3e3e;
        }
    """)

class AdminGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Admin Dashboard")
        self.setGeometry(150, 150, 1400, 900)

        apply_kaki_theme(self)

        central = QWidget()
        layout = QVBoxLayout()

        self.monitor_panel = MonitorPanel()
        apply_kaki_theme(self.monitor_panel)

        self.monitor_panel.btn_robots.clicked.connect(self.open_robot_dialog)
        self.monitor_panel.btn_requests.clicked.connect(self.open_request_dialog)
        self.monitor_panel.btn_logs.clicked.connect(self.open_log_dialog)

        layout.addWidget(self.monitor_panel)
        central.setLayout(layout)
        self.setCentralWidget(central)

    def open_log_dialog(self):
        dialog = LogDialog()
        dialog.exec_()

    def open_robot_dialog(self):
        dialog = RobotDialog()
        dialog.exec_()

    def open_request_dialog(self):
        dialog = RequestDialog()
        dialog.exec_()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AdminGUI()
    window.show()
    sys.exit(app.exec_())