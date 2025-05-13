import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2

class UdpStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # AI 서버 정보 (변경 가능)
        self.server_ip = '192.168.0.168'
        self.server_port = 5005
        self.get_logger().info(f"UDP 전송 노드 시작됨 → {self.server_ip}:{self.server_port}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', frame)
            data = jpeg.tobytes()

            # 안전을 위해 64KB 이상은 생략
            if len(data) > 64000:
                self.get_logger().warn("전송 이미지 크기 초과, 생략됨")
                return

            self.sock.sendto(data, (self.server_ip, self.server_port))
        except Exception as e:
            self.get_logger().error(f"UDP 전송 중 오류: {e}")

def main():
    rclpy.init()
    node = UdpStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
