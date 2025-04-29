# viewer/staff/request_wait_panel.py

from PyQt6.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout
from PyQt6.QtCore import Qt
from viewer.theme import apply_theme

class RequestWaitPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent  # StaffGUI (메인 창) 연결
        apply_theme(self)

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.status_label = QLabel("요청중...")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("font-size: 24px; font-weight: bold;")
        layout.addWidget(self.status_label)

        self.btn_cancel = QPushButton("취소")
        self.btn_cancel.setFixedSize(200, 50)
        self.btn_cancel.clicked.connect(self.on_cancel_clicked)
        layout.addWidget(self.btn_cancel)

        self.setLayout(layout)

    def update_status(self, status_text):
        """상태 텍스트를 업데이트하고 버튼 상태도 조정"""
        self.status_label.setText(status_text)

        # 상태에 따라 버튼 활성화/비활성화
        if status_text in ["요청중...", "이동중..."]:
            self.btn_cancel.setEnabled(True)
        else:
            self.btn_cancel.setEnabled(False)

    def on_cancel_clicked(self):
        print("요청 취소 버튼 클릭됨")
        # 취소하면 다시 카메라로 이동
        self.parent_gui.go_to_camera()
