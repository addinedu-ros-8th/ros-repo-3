import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QStackedWidget
from PyQt6.QtCore import QTimer  # ⬅️ 추가
from viewer.theme import apply_theme

from .camera_panel import CameraPanel
from .product_info_panel import ProductInfoPanel
from .cart_panel import CartPanel
from .request_wait_panel import RequestWaitPanel
from viewer.staff.cache_manager import CacheManager
from viewer.staff.tcp_sender import TCPClientThread


class StaffGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Staff GUI")
        self.setFixedSize(480, 800)
        #self.setGeometry(100, 100, 480, 800)
        apply_theme(self)

        self.central_widget = QWidget()
        self.main_layout = QVBoxLayout(self.central_widget)
        self.cache_manager = CacheManager()

        # TCP 클라이언트 시작
        self.tcp_client = TCPClientThread(host="127.0.0.1", port=9000)
        self.tcp_client.received.connect(self.on_server_response)
        self.tcp_client.start()

        # 스택 UI 구성
        self.stack = QStackedWidget()
        self.main_layout.addWidget(self.stack)
        self.setCentralWidget(self.central_widget)

        self.camera_panel = CameraPanel(parent=self)
        self.product_info_panel = ProductInfoPanel(parent=self)
        self.cart_panel = CartPanel(parent=self)
        self.request_wait_panel = RequestWaitPanel(parent=self)

        self.stack.addWidget(self.camera_panel)
        self.stack.addWidget(self.product_info_panel)
        self.stack.addWidget(self.cart_panel)
        self.stack.addWidget(self.request_wait_panel)

        self.go_to_camera()

        # ⬇️ 프로그램 시작 후 일정 시간 뒤 배송 취소 요청 자동 전송
        QTimer.singleShot(1000, self.send_test_cancel_delivery)

    def go_to_camera(self):
        self.camera_panel.reset_qr_detection()
        self.stack.setCurrentIndex(0)

    def go_to_product_info(self, product_data=None):
        if product_data:
            self.product_info_panel.update_product_info(product_data)
        self.stack.setCurrentIndex(1)

    def go_to_cart(self):
        self.cart_panel.load_cart_items()
        self.stack.setCurrentIndex(2)

    def go_to_request_wait(self, status_text="요청중..."):
        self.request_wait_panel.update_status(status_text)
        self.stack.setCurrentIndex(3)

    # ✅ 서버로 메시지 전송하는 일반 메서드
    def send_message_to_server(self, msg: str):
        self.tcp_client.send(msg)

    # ✅ 응답 처리
    def on_server_response(self, msg: bytes):
        print(f"[서버 응답] {msg}")

    # ✅ 테스트용 배송 취소 요청
    def send_test_cancel_delivery(self):
        user_id = 7
        delivery_id = 1234
        print(f"[전송] 배송 취소 요청: user_id={user_id}, delivery_id={delivery_id}")
        self.tcp_client.send_cancel_delivery_request(user_id, delivery_id)

    def closeEvent(self, event):
        self.tcp_client.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StaffGUI()
    window.show()
    sys.exit(app.exec())
