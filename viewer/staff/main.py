# viewer/staff/main.py

import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QStackedWidget
from viewer.theme import apply_theme

# 나중에 실제 파일 연결할 예정
from .camera_panel import CameraPanel
from .product_info_panel import ProductInfoPanel
from .cart_panel import CartPanel
from .request_wait_panel import RequestWaitPanel

class StaffGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Staff GUI")
        self.setGeometry(100, 100, 480, 800)  # 모바일 사이즈로 수정
        apply_theme(self)

        self.central_widget = QWidget()
        self.main_layout = QVBoxLayout(self.central_widget)

        # 스택 위젯 생성
        self.stack = QStackedWidget()
        self.main_layout.addWidget(self.stack)

        self.setCentralWidget(self.central_widget)

        # 각 화면 패널 초기화
        self.camera_panel = CameraPanel(parent=self)
        self.product_info_panel = ProductInfoPanel(parent=self)
        self.cart_panel = CartPanel(parent=self)
        self.request_wait_panel = RequestWaitPanel(parent=self)

        # 스택에 화면 추가
        self.stack.addWidget(self.camera_panel)         # Index 0
        self.stack.addWidget(self.product_info_panel)   # Index 1
        self.stack.addWidget(self.cart_panel)           # Index 2
        self.stack.addWidget(self.request_wait_panel)   # Index 3

        # 초기 화면: 카메라
        self.go_to_camera()

    # 화면 전환 메소드들
    def go_to_camera(self):
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

    def go_to_camera(self):
        self.camera_panel.reset_qr_detection()  # <-- 추가
        self.stack.setCurrentIndex(0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StaffGUI()
    window.show()
    sys.exit(app.exec())
