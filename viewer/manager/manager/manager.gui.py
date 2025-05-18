import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox
from viewer.UI.manager_login_ui import Ui_MainWindow
from viewer.manager.manager.main import MainWindow as ManagerMainWindow
from db.connect_db import get_user_info
from viewer.theme import apply_theme


class ManagerLogin(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Manager Login")
        apply_theme(self)

        self.ui.login_btn.clicked.connect(self.try_login)

    def try_login(self):
        user_id = self.ui.id_input.text()
        password = self.ui.pw_input.text()

        user = get_user_info(user_id, password)
        if user and user["role"] == "manager":
            self.manager_window = ManagerMainWindow()
            self.manager_window.show()
            self.close()
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid ID/PW or Role")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ManagerLogin()
    window.show()
    sys.exit(app.exec())
