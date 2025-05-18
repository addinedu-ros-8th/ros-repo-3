from PyQt6.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout
from PyQt6.QtCore import Qt
from viewer.theme import apply_theme
from viewer.staff.tcp_sender import TCPClientThread
from viewer.staff.message_router import MessageRouter

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
            self.send_inventory_request()
        else:
            self.btn_cancel.setEnabled(False)

    def send_inventory_request(self):
        # 테스트용 user_id 하드코딩
        user_id = 1

        # 실제 유저 ID를 사용하는 경우
        # user_id = self.parent_gui.user_id if hasattr(self.parent_gui, 'user_id') else 0

        destination = "G1"  # 목적지 고정 또는 다른 로직으로 설정 가능
        items = self.parent_gui.cache_manager.get_items() if hasattr(self.parent_gui, 'cache_manager') else []

        print(f"[RequestWaitPanel] 장바구니 항목 개수: {len(items)}")
        for idx, item in enumerate(items):
            print(f"[RequestWaitPanel] item[{idx}]: {item}")

        if not items:
            print("[RequestWaitPanel] ⚠️ 장바구니가 비어 있습니다. 요청 중단")
            return

        if hasattr(self.parent_gui, 'tcp_client'):
            self.parent_gui.tcp_client.send_inventory_request(user_id, destination, items)
            print("[RequestWaitPanel] ✅ 서버로 요청 전송 완료")
        else:
            print("[오류] TCP 클라이언트가 설정되지 않았습니다.")

    def on_inventory_response_success(self):
        """서버 응답이 0x00인 경우 호출: 작업 상태 화면으로 이동"""
        self.parent_gui.go_to_task_status()

    def on_cancel_clicked(self):
        print("요청 취소 버튼 클릭됨")
        # 취소하면 다시 카메라로 이동
        self.parent_gui.go_to_camera()