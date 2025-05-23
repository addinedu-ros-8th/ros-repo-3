# viewer/staff/main.py
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QStackedWidget
from gui.staff.view.base_panel import BasePanel
from gui.staff.view.cache_manager import CacheManager
from gui.staff.comm.tcp_sender import TCPClientThread
from gui.staff.comm.message_router import MessageRouter
from gui.staff.view.staff_login import LoginWindow

class MainWindow(QMainWindow):
    def __init__(self, tcp_client, user_id, user_role):
        super().__init__()
        # 캐시 매니저 초기화
        self.cache_manager = CacheManager()

        self.tcp_client = tcp_client
        self.user_id = user_id
        self.user_role = user_role

        self.setWindowTitle("Staff GUI")
        self.setFixedSize(480, 800)

        container = BasePanel()
        self.stack = QStackedWidget(container)
        layout = QVBoxLayout(container)
        layout.addWidget(self.stack)
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.router = MessageRouter(parent_gui=self)
        self.tcp_client.received.connect(self.router.handle_response)

        self._panels = {}
        self.go_to_camera()

    def go_to_camera(self):
        if 'camera' not in self._panels:
            from gui.staff.view.camera_panel import CameraPanel
            panel = CameraPanel(tcp_thread=self.tcp_client, parent=self, main_window=self)
            self._panels['camera'] = panel
            self.stack.addWidget(panel)
        panel = self._panels['camera']
        panel.reset_qr_detection()
        self.stack.setCurrentWidget(panel)

    def go_to_product_info(self, product_data=None):
        if 'product' not in self._panels:
            from gui.staff.view.product_info_panel import ProductInfoPanel
            panel = ProductInfoPanel(parent=self, main_window=self)
            self._panels['product'] = panel
            self.stack.addWidget(panel)
        panel = self._panels['product']
        if product_data:
            panel.update_product_info(product_data)
        self.stack.setCurrentWidget(panel)

    def go_to_cart(self):
        if 'cart' not in self._panels:
            from gui.staff.view.cart_panel import CartPanel
            panel = CartPanel(parent=self)
            self._panels['cart'] = panel
            self.stack.addWidget(panel)
        panel = self._panels['cart']
        panel.load_cart_items()
        self.stack.setCurrentWidget(panel)

    def go_to_request_wait(self, status_text="요청중..."):
        if 'request' not in self._panels:
            from gui.staff.view.request_wait_panel import RequestWaitPanel
            panel = RequestWaitPanel(parent=self)
            self._panels['request'] = panel
            self.stack.addWidget(panel)
        panel = self._panels['request']
        panel.update_status(status_text)
        self.stack.setCurrentWidget(panel)

    def go_to_task_status(self):
        if 'task' not in self._panels:
            from gui.staff.comm.task_status_cancel import TaskStatusPanel
            panel = TaskStatusPanel(tcp_thread=self.tcp_client,
                                    main_window=self,
                                    user_id=self.user_id,
                                    parent=self)
            self._panels['task'] = panel
            self.stack.addWidget(panel)
        panel = self._panels['task']
        print("[MainWindow] TaskStatusPanel로 전환, 조회 요청 보냄")
        panel.load_task_status()
        self.stack.setCurrentWidget(panel)

    def closeEvent(self, event):
        self.tcp_client.stop()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    tcp = TCPClientThread(host="127.0.0.1", port=9000)
    tcp.start()

    login = LoginWindow(tcp_thread=tcp)
    login.show()
    def on_login(user_id, user_role):
        login.close()
        win = MainWindow(tcp, user_id, user_role)
        win.show()
    login.loginSuccess.connect(on_login)
    sys.exit(app.exec())
