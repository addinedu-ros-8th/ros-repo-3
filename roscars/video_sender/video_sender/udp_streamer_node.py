import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2
import numpy as np
import struct

class UdpStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer_node')

        # AI 서버 정보
        self.server_ip = '192.168.0.168'
        self.server_port = 4436
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 이 로봇의 ID (0~255)
        self.robot_id = 1

        self.get_logger().info(f"UDP 전송 노드 시작됨 → {self.server_ip}:{self.server_port}")

    def image_callback(self, msg):
        try:
            # 1. ROS Image → OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 2. 해상도 축소 + JPEG 압축
            resized = cv2.resize(frame, (320, 240))
            ok, jpeg = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
            if not ok:
                self.get_logger().warn("JPEG 인코딩 실패")
                return

            body = jpeg.tobytes()
            body_len = len(body)
            self.get_logger().info(f"전송 크기: {body_len} bytes")

            # 3. 헤더(1B id + 4B length) + body
            header = struct.pack('!BI', self.robot_id, body_len)
            packet = header + body

            # 4. UDP 전송 (MTU 고려)
            if len(packet) > 65507:
                self.get_logger().warn("패킷 크기 초과, 생략")
                return

            self.sock.sendto(packet, (self.server_ip, self.server_port))

        except Exception as e:
            self.get_logger().error(f"UDP 전송 중 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UdpStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료됨 (Ctrl+C)")
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()
