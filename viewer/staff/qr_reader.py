# viewer/staff/qr_reader.py

import cv2
from pyzbar.pyzbar import decode

def decode_qr(frame):
    """OpenCV 프레임에서 QR코드를 읽어 텍스트로 반환"""
    decoded_objects = decode(frame)
    for obj in decoded_objects:
        return obj.data.decode('utf-8')  # 첫 번째 QR 코드 데이터 반환
    return None  # QR 코드 없으면 None
