import os
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QMessageBox
from PyQt6 import uic
from gui.shared.theme import apply_theme
from gui.manager.view.dashboard_panel import MonitorPanel

ui_path = os.path.join("gui/manager/view_ui/manager_login.ui")
from_class = uic.loadUiType(ui_path)[0]

class ManagerMainWindow(QMainWindow):
    def __init__(self, ros_interface):
        super().__init__()
        self.setWindowTitle("Shoepernoma Manager Dashboard")
        self.setGeometry(150, 150, 1400, 900)
        apply_theme(self)

        central = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(MonitorPanel(ros_interface))
        central.setLayout(layout)
        self.setCentralWidget(central)

    def show_message(self, title, message):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Icon.Information)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec()

class ManagerLoginWindow(QMainWindow, from_class):
    def __init__(self, dashboard_ros):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Manager Login Display")
        apply_theme(self)

        self.ros = dashboard_ros
        self.ros.login_response.connect(self.handle_login_result)

        self.login_btn.clicked.connect(self.try_login)
        self.pw_input.returnPressed.connect(self.try_login)

    def try_login(self):
        user_name = self.id_input.text()
        password = self.pw_input.text()
        self.ros.request_login(user_name, password)

    def handle_login_result(self, success, role, user_id):
        print(success)
        if success:
            if role == "MANAGER":
                self.manager_window = ManagerMainWindow(self.ros)
                self.manager_window.show()
                self.close()
            elif role == "STAFF":
                QMessageBox.warning(self, "로그인 실패", "Staff gui로 로그인해주세요.")
        else:
            QMessageBox.warning(self, "로그인 실패", "아이디 또는 비밀번호가 올바르지 않습니다.")
