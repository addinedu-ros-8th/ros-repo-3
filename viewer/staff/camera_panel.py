from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QMessageBox
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap
import cv2
import time

from viewer.staff.qr_reader import decode_qr
from viewer.staff.tcp_sender import TCPClientThread
from viewer.staff.message_router import MessageRouter


class CameraPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent
        self.capture = cv2.VideoCapture(0)
        self.last_detected_qr = None

        self.tcp_thread = TCPClientThread()
        self.message_router = MessageRouter(None, parent_gui=self.parent_gui)

        self.tcp_thread.received.connect(self.message_router.handle_response)
        self.tcp_thread.start()

        self.init_ui()
        self.wait_for_camera()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def wait_for_camera(self):
        attempts = 0
        while not self.capture.isOpened() and attempts < 5:
            print("카메라 초기화 중...")
            time.sleep(1)
            self.capture.open(0)
            attempts += 1
        print("카메라 초기화 완료!" if self.capture.isOpened() else "카메라 초기화 실패")

    def init_ui(self):
        layout = QVBoxLayout()
        self.camera_label = QLabel("카메라 로딩 중...")
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.camera_label)
        self.setLayout(layout)

    def update_frame(self):
        ret, frame = self.capture.read()
        if ret:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(qt_image).scaled(
                self.camera_label.width(), self.camera_label.height(), Qt.AspectRatioMode.KeepAspectRatio
            ))

            qr_data = decode_qr(frame)
            if qr_data and qr_data != self.last_detected_qr:
                print(f"Detected QR: {qr_data}")
                self.last_detected_qr = qr_data
                self.tcp_thread.send_item_info_request(qr_data)

    def closeEvent(self, event):
        if self.capture.isOpened():
            self.capture.release()
        self.tcp_thread.stop()
        event.accept()

    def reset_qr_detection(self):
        self.last_detected_qr = None