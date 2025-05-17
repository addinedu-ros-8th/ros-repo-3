from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap
import cv2
import time

# QR 코드 디코더
from viewer.staff.qr_reader import decode_qr


class CameraPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent  # StaffGUI (메인 창) 연결
        self.capture = cv2.VideoCapture(0)  # 기본 카메라(0번)

        self.last_detected_qr = None  # 최근 감지된 QR 코드 기억

        self.init_ui()

        # 카메라 초기화 대기
        self.wait_for_camera()

        # 타이머를 통해 프레임 업데이트
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 약 30ms마다 프레임 업데이트

    def wait_for_camera(self):
        """카메라가 열릴 때까지 대기"""
        attempts = 0
        while not self.capture.isOpened() and attempts < 5:
            print("카메라 초기화 중...")
            time.sleep(1)
            self.capture.open(0)
            attempts += 1
        if self.capture.isOpened():
            print("카메라 초기화 완료!")
        else:
            print("카메라 초기화 실패")

    def init_ui(self):
        layout = QVBoxLayout()

        self.camera_label = QLabel("카메라 로딩 중...")
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.camera_label)

        self.setLayout(layout)

    def update_frame(self):
        """카메라 프레임을 받아와서 화면에 표시"""
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

            # QR 코드 인식 처리
            qr_data = decode_qr(frame)
            if qr_data and qr_data != self.last_detected_qr:
                print(f"Detected QR: {qr_data}")
                self.last_detected_qr = qr_data

                try:
                    # QR 문자열 포맷: "{name} | {size} | {color} | {location_name} | {quantity}"
                    name, size, color, rack, quantity = [item.strip() for item in qr_data.split("|")]

                    product_data = {
                        "qr_data": qr_data,
                        "model": name,
                        "size": size,
                        "color": color,
                        "rack": rack,
                        "quantity": int(quantity)
                    }

                    self.parent_gui.go_to_product_info(product_data)

                except ValueError:
                    print("❌ QR 코드 파싱 실패: 형식이 예상과 다릅니다.")

    def closeEvent(self, event):
        """창 닫을 때 카메라 해제"""
        if self.capture.isOpened():
            self.capture.release()
        event.accept()

    def reset_qr_detection(self):
        """QR 인식 기록을 리셋해서 다시 읽을 수 있게 한다."""
        self.last_detected_qr = None
