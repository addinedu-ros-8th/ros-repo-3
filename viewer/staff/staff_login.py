import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox
from viewer.UI.staff_login_ui import Ui_MainWindow
from viewer.staff.main import StaffGUI
from db.connect_db import get_user_info  # 사용자 인증 함수
from viewer.theme import apply_theme


class StaffLogin(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Staff Login")
        apply_theme(self)

        self.ui.login_btn.clicked.connect(self.try_login)

    def try_login(self):
        user_id = self.ui.id_input.text()
        password = self.ui.pw_input.text()

        user = get_user_info(user_id, password)
        if user and user["role"] == "staff":
            self.staff_window = StaffGUI()
            self.staff_window.show()
            self.close()
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid ID/PW or Role")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StaffLogin()
    window.show()
    sys.exit(app.exec())
