import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2
import numpy as np

class UdpStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer_node')

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

        self.get_logger().info(f"UDP 전송 노드 시작됨 → {self.server_ip}:{self.server_port}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 해상도 축소 및 압축
            resized = cv2.resize(frame, (320, 240))
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
            success, jpeg = cv2.imencode('.jpg', resized, encode_param)

            if not success:
                self.get_logger().warn("JPEG 인코딩 실패")
                return

            data = jpeg.tobytes()
            data_len = len(data)
            self.get_logger().info(f"전송 크기: {data_len} bytes")

            if data_len > 64000:
                self.get_logger().warn("전송 이미지 크기 초과 (생략됨)")
                return

            self.sock.sendto(data, (self.server_ip, self.server_port))

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
