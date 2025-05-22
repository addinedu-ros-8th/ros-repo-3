from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QHBoxLayout, QMessageBox
from PyQt6.QtCore import Qt, pyqtSignal
from gui.theme import apply_theme
from gui.staff.message_router import MessageRouter

class LoginWindow(QMainWindow):
    loginSuccess = pyqtSignal(int, str)

    def __init__(self, tcp_thread):
        super().__init__()
        self.tcp_client = tcp_thread

        self.setWindowTitle("Login")
        self.setFixedSize(400, 300)
        apply_theme(self)

        self.router = MessageRouter(parent_gui=self)
        self.tcp_client.received.connect(self.router.handle_response)

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        title = QLabel("Staff Login", self)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        self.id_input = QLineEdit(self)
        self.id_input.setPlaceholderText("ID")
        layout.addWidget(self.id_input)

        self.pw_input = QLineEdit(self)
        self.pw_input.setPlaceholderText("Password")
        self.pw_input.setEchoMode(QLineEdit.EchoMode.Password)
        layout.addWidget(self.pw_input)

        btn_layout = QHBoxLayout()
        login_btn = QPushButton("Login", self)
        login_btn.clicked.connect(self.on_login)
        btn_layout.addStretch()
        btn_layout.addWidget(login_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def on_login(self):
        user = self.id_input.text().strip()
        pw = self.pw_input.text()
        if not user or not pw:
            QMessageBox.warning(self, "입력 오류", "ID와 Password를 모두 입력해주세요.")
            return
        self.tcp_client.send_login_request(user, pw)

    def on_login_success(self, user_id, user_role_code):
        role = "STAFF" if user_role_code==1 else "MANAGER"
        QMessageBox.information(self, "로그인 성공", f"환영합니다! ID: {user_id}, 역할: {role}")
        self.loginSuccess.emit(user_id, role)
        self.close()