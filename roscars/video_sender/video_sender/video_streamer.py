#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from picamera2 import Picamera2
import socket
import cv2
import struct
import zlib
import os
import re
import threading
import queue
import time

MAX_UDP_SIZE = 65507  # 안전한 페이로드 한계

class UdpStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer')

        # 1) 파라미터 선언 및 로드
        self.declare_parameter('server_ip', '192.168.0.30')
        self.declare_parameter('server_port', 4436)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('robot_ssid', self.get_ssid_from_hostapd())

        p = self.get_parameter
        self.server_ip   = p('server_ip').get_parameter_value().string_value
        self.server_port = p('server_port').get_parameter_value().integer_value
        self.quality     = p('jpeg_quality').get_parameter_value().integer_value
        self.robot_ssid  = p('robot_ssid').get_parameter_value().string_value

        # 2) Picamera2 설정
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "XRGB8888", "size": (640, 480)}
        )
        self.picam2.configure(config)
        self.picam2.start()

        # 3) UDP 소켓 연결
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.connect((self.server_ip, self.server_port))
            self.get_logger().info(f"UDP 연결 완료 → {self.server_ip}:{self.server_port}")
        except Exception as e:
            self.get_logger().error(f"UDP 연결 실패: {e}")

        # 4) 프레임 큐 및 전송 스레드
        self.frame_queue = queue.Queue(maxsize=2)
        threading.Thread(target=self._send_loop, daemon=True).start()

        # 5) 주기적 캡처 타이머 (10Hz)
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"UDP 송신 준비 → ssid={self.robot_ssid}, q={self.quality}")

    def get_ssid_from_hostapd(self):
        """hostapd.conf에서 SSID 파싱 (AP 모드용)"""
        paths = [
            "/etc/hostapd/hostapd.conf",
            "/etc/hostapd.conf",
            "/etc/network/hostapd.conf",
            "/home/pinky/robot_ap.conf"
        ]
        for path in paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        for line in f:
                            m = re.match(r'^ssid=(.+)', line.strip())
                            if m:
                                return m.group(1).strip()
                except:
                    pass
        return "unknown"

    def timer_callback(self):
        """Picamera2로부터 프레임 캡처 후 큐에 추가"""
        try:
            rgb_frame = self.picam2.capture_array()
            # OpenCV는 BGR, Picamera2는 RGB
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            if not self.frame_queue.full():
                self.frame_queue.put(frame)
        except Exception as e:
            self.get_logger().error(f"캡처 오류: {e}")

    def _send_loop(self):
        """큐에서 프레임 꺼내 압축→패킷화→UDP 전송"""
        while rclpy.ok():
            frame = self.frame_queue.get()
            try:
                ok, buf = cv2.imencode('.jpg', frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
                if not ok:
                    self.get_logger().warn("JPEG 인코딩 실패")
                    continue

                comp = zlib.compress(buf.tobytes())
                comp_len = len(comp)
                ssid_b = self.robot_ssid.encode('utf-8')
                ssid_len = len(ssid_b)

                # 패킷: [1B ssid_len][ssid][4B comp_len][data]
                header = struct.pack(f"!B{ssid_len}sI",
                                     ssid_len, ssid_b, comp_len)
                packet = header + comp

                if len(packet) > MAX_UDP_SIZE:
                    self.get_logger().warn(
                        f"패킷 크기 초과 ({len(packet)}/{MAX_UDP_SIZE}), 생략")
                    continue

                self.sock.send(packet)
            except Exception as e:
                self.get_logger().error(f"전송 오류: {e}")
            finally:
                # 과도한 CPU 점유 방지
                time.sleep(0.001)

    def destroy_node(self):
        """노드 종료 시 리소스 정리"""
        try:
            self.sock.close()
            self.picam2.stop()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UdpStreamer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
