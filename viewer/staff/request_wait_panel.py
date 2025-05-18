from PyQt6.QtWidgets import QVBoxLayout, QLabel, QPushButton
from PyQt6.QtCore import Qt
from viewer.staff.base_panel import BasePanel

class RequestWaitPanel(BasePanel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent
        # 서버 요청 성공 시 화면 전환을 위해 시그널 연결
        self.parent_gui.router.inventoryRequestSuccess.connect(self.on_inventory_response_success)
        self.parent_gui.router.inventoryRequestFailure.connect(self._on_request_failure)
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        self.status_label = QLabel("요청중...", self)
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status_label)

        cancel_btn = QPushButton("취소", self)
        cancel_btn.clicked.connect(self._on_cancel)
        layout.addWidget(cancel_btn)

        self.setLayout(layout)

    def showEvent(self, event):
        super().showEvent(event)
        # 패널이 보일 때 요청 전송
        if self.status_label.text() in ["요청중...", "이동중..."]:
            self._send_request()

    def update_status(self, text):
        self.status_label.setText(text)
        if self.isVisible() and text in ["요청중...", "이동중..."]:
            self._send_request()

    def _send_request(self):
        uid = getattr(self.parent_gui, 'user_id', 0)
        dest = "G1"
        items = self.parent_gui.cache_manager.get_items()
        if not items:
            print("[RequestWaitPanel] ⚠️ 장바구니가 비어 있습니다.")
            return

        print(f"[RequestWaitPanel] 서버로 요청 전송: uid={uid}, dest={dest}, items={items}")
        self.parent_gui.tcp_client.send_inventory_request(uid, dest, items)

    def on_inventory_response_success(self):
        # 서버로부터 요청 성공 응답을 받으면 작업 상태 화면으로 이동
        self.parent_gui.go_to_task_status()

    def _on_request_failure(self):
        # 요청 실패 시 상태 텍스트 업데이트
        self.status_label.setText("요청 실패, 다시 시도해주세요.")

    def _on_cancel(self):
        self.parent_gui.go_to_camera()