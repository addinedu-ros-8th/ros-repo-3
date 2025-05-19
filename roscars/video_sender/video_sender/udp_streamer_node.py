import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2
import struct
import zlib
import os
import re

MAX_UDP_SIZE = 65507  # 안전한 페이로드 한계

class UdpStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer_node')

        # 파라미터
        self.declare_parameter('server_ip', '192.168.0.30')
        self.declare_parameter('server_port', 4436)
        self.declare_parameter('robot_ssid', self.get_ssid_from_hostapd())
        self.declare_parameter('jpeg_quality', 60)
        p = self.get_parameter

        self.server_ip = p('server_ip').get_parameter_value().string_value
        self.server_port = p('server_port').get_parameter_value().integer_value
        self.robot_ssid = p('robot_ssid').get_parameter_value().string_value
        self.quality = p('jpeg_quality').get_parameter_value().integer_value

        # UDP 소켓 연결
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.connect((self.server_ip, self.server_port))
            self.get_logger().info(f"UDP 연결 완료 → {self.server_ip}:{self.server_port}")
        except Exception as e:
            self.get_logger().error(f"UDP 연결 실패: {e}")

        # 이미지 수신 구독자
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
        self.latest_msg = None

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
                            match = re.match(r'^ssid=(.+)', line.strip())
                            if match:
                                return match.group(1).strip()
                except Exception:
                    continue
        return "unknown"

    def cb(self, msg: Image):
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_msg, 'bgr8')
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok:
                self.get_logger().warn("JPEG 인코딩 실패")
                return

            comp = zlib.compress(buf.tobytes())
            comp_len = len(comp)

            ssid_bytes = self.robot_ssid.encode('utf-8')
            ssid_len = len(ssid_bytes)

            if ssid_len > 255:
                self.get_logger().error("SSID가 너무 깁니다 (최대 255바이트)")
                return

            # 패킷 구성: 1바이트 ssid 길이 + ssid + 4바이트 압축 길이 + 압축된 데이터
            packet = struct.pack(f"!B{ssid_len}sI", ssid_len, ssid_bytes, comp_len) + comp

            if len(packet) > MAX_UDP_SIZE:
                self.get_logger().warn(f"패킷 크기 초과 ({len(packet)}/{MAX_UDP_SIZE}), 생략")
                return

            self.sock.send(packet)
            self.get_logger().debug(f"전송: {self.robot_ssid}, 압축 {comp_len} 바이트")

        except Exception as e:
            self.get_logger().error(f"전송 오류: {e}")

    def destroy_node(self):
        try:
            self.sock.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UdpStreamer()
    node.create_timer(0.1, node.timer_callback)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
