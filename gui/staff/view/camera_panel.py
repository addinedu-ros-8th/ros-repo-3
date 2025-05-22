from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QVBoxLayout, QHBoxLayout, QLabel, QPushButton
import cv2, time
from gui.staff.base_panel import BasePanel
from gui.staff.qr_reader import decode_qr

class CameraPanel(BasePanel):
    def __init__(self, tcp_thread, main_window, parent=None):
        super().__init__(parent)
        self.tcp_thread = tcp_thread
        self.main_window = main_window
        self.capture = cv2.VideoCapture(0)
        self.last_detected_qr = None

        self._init_ui()
        self._wait_for_camera()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_frame)
        self.timer.start(30)

    def _wait_for_camera(self):
        for _ in range(5):
            if self.capture.isOpened():
                break
            time.sleep(1)
        print("카메라 초기화 완료!" if self.capture.isOpened() else "카메라 초기화 실패")

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # 카메라 프레임 표시용 라벨
        self.camera_label = QLabel("카메라 로딩 중...", self)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.camera_label)

        # 하단 버튼 영역
        btn_layout = QHBoxLayout()
        cart_btn = QPushButton("장바구니", self)
        cart_btn.clicked.connect(self._go_to_product_info_panel)

        task_btn = QPushButton("작업현황", self)
        task_btn.clicked.connect(self._go_to_task_status_panel)

        btn_layout.addWidget(cart_btn)
        btn_layout.addWidget(task_btn)
        layout.addLayout(btn_layout)

    def _update_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bpl = ch * w
        img = QImage(rgb.data, w, h, bpl, QImage.Format.Format_RGB888)
        pix = QPixmap.fromImage(img).scaled(
            self.camera_label.width(),
            self.camera_label.height(),
            Qt.AspectRatioMode.KeepAspectRatio
        )
        self.camera_label.setPixmap(pix)

        qr = decode_qr(frame)
        if qr and qr != self.last_detected_qr:
            self.last_detected_qr = qr
            self.tcp_thread.send_item_info_request(qr)

    def reset_qr_detection(self):
        self.last_detected_qr = None

    def hideEvent(self, event):
        """화면이 다른 패널로 넘어갈 때 카메라를 꺼준다."""
        if self.capture.isOpened():
            self.capture.release()
            print("카메라 종료됨")
        event.accept()

    def showEvent(self, event):
        """다시 화면에 보일 때 카메라를 재시작한다."""
        if not self.capture.isOpened():
            self.capture = cv2.VideoCapture(0)
            self._wait_for_camera()
        if not self.timer.isActive():
            self.timer.start(30)
        event.accept()

    def _go_to_product_info_panel(self):
        self.main_window.go_to_product_info()

    def _go_to_task_status_panel(self):
        self.main_window.go_to_task_status()
