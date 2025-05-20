import sys
import os
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt6 import uic
from viewer.manager.manager.main import MainWindow as ManagerMainWindow
from viewer.theme import apply_theme  # 테마 적용

# .ui 파일 경로
base_dir = os.path.dirname(__file__)
ui_path = os.path.join(base_dir, "manager_login.ui")
from_class = uic.loadUiType(ui_path)[0]

class windowsClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Manager Login Display")
        apply_theme(self)

        # 로그인 버튼 시그널 연결
        self.login_btn.clicked.connect(self.try_login)

        # 엔터키로 로그인 동작
        self.pw_input.returnPressed.connect(self.try_login)

    def try_login(self):
        username = self.id_input.text()
        password = self.pw_input.text()

        # 하드코딩된 테스트 계정
        if username == "admin" and password == "1234":
            self.open_manager_gui()
        else:
            QMessageBox.warning(self, "로그인 실패", "아이디 또는 비밀번호가 올바르지 않습니다.")

    def open_manager_gui(self):
        self.manager_window = ManagerMainWindow()
        self.manager_window.show()
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = windowsClass()
    myWindows.show()
    sys.exit(app.exec())
