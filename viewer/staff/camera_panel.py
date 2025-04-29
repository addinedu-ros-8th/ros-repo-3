# viewer/staff/camera_panel.py

from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap
import cv2

# 추가: QR 코드 인식 모듈 가져오기
from viewer.staff.qr_reader import decode_qr

class CameraPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent  # StaffGUI (메인 창) 연결
        self.capture = cv2.VideoCapture(0)  # 기본 카메라(0번)

        self.last_detected_qr = None  # 최근 감지된 QR 코드 기억

        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 약 30ms마다 프레임 업데이트

    def init_ui(self):
        layout = QVBoxLayout()

        self.camera_label = QLabel("카메라 로딩 중...")
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.camera_label)

        self.setLayout(layout)

    def update_frame(self):
        ret, frame = self.capture.read()
        if ret:
            # OpenCV 이미지를 Qt 이미지로 변환
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

            self.camera_label.setPixmap(QPixmap.fromImage(qt_image).scaled(
                self.camera_label.width(), self.camera_label.height(), Qt.AspectRatioMode.KeepAspectRatio
            ))

            # ➡ QR 코드 인식 추가
            qr_data = decode_qr(frame)
            if qr_data and qr_data != self.last_detected_qr:
                print(f"Detected QR: {qr_data}")  # 디버깅용 출력
                self.last_detected_qr = qr_data

                # (나중에 DB 조회 붙이기 전까지는) 단순히 QR 코드 데이터 넘기기
                product_data = {
                    "qr_data": qr_data,
                    "model": "임시모델",
                    "color": "블랙",
                    "size": "260",
                    "rack": "A1",
                    "quantity": 5
                }
                self.parent_gui.go_to_product_info(product_data)

    def closeEvent(self, event):
        if self.capture.isOpened():
            self.capture.release()
        event.accept()

    def reset_qr_detection(self):
        """QR 인식 기록을 리셋해서 다시 읽을 수 있게 한다."""
        self.last_detected_qr = None
