import cv2
import socket
import struct
import zlib

class VideoStreamer:
    """JPEG 압축 + zlib 압축 → UDP 전송"""
    def __init__(self, server_ip: str, server_port: int, quality: int = 60):
        self.server_ip   = server_ip
        self.server_port = server_port
        self.quality     = quality
        self.sock        = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame, ssid: str):
        # JPEG 인코딩
        ret, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ret:
            return

        # zlib 압축
        data = zlib.compress(buf.tobytes(), level=9)

        # SSID 바이트 & 길이
        ssid_b = ssid.encode('utf-8')
        ssid_len = len(ssid_b)

        # 전체 압축 데이터 길이
        comp_len = len(data)

        # 헤더: [1B ssid_len][ssid][4B comp_len]
        header = struct.pack(f"!B{ssid_len}sI", ssid_len, ssid_b, comp_len)
        packet = header + data
        self.sock.sendto(packet, (self.server_ip, self.server_port))