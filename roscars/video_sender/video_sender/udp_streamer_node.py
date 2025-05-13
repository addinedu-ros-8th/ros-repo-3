import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2
import struct
import zlib

MAX_UDP_SIZE = 65507  # 안전한 페이로드 한계

class UdpStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer_node')
        # 파라미터 선언
        self.declare_parameter('server_ip', '192.168.0.168')
        self.declare_parameter('server_port', 4436)
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('jpeg_quality', 60)  # 테스트하신 60
        p = self.get_parameter
        self.server_ip   = p('server_ip').get_parameter_value().string_value
        self.server_port = p('server_port').get_parameter_value().integer_value
        self.robot_id    = p('robot_id').get_parameter_value().integer_value
        self.quality     = p('jpeg_quality').get_parameter_value().integer_value

        # UDP 소켓
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # CvBridge, 구독
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
        self.latest_msg = None

        self.get_logger().info(
            f"UDP 송신 시작 → {self.server_ip}:{self.server_port}, id={self.robot_id}, q={self.quality}"
        )

    def cb(self, msg: Image):
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is None:
            return
        try:
            # Image → CV frame
            frame = self.bridge.imgmsg_to_cv2(self.latest_msg, 'bgr8')
            # JPEG 인코딩
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok:
                self.get_logger().warn("JPEG 인코딩 실패")
                return

            # zlib 압축
            comp = zlib.compress(buf.tobytes())
            comp_len = len(comp)
            # header: 1B id + 4B length
            hdr = struct.pack('!BI', self.robot_id, comp_len)
            packet = hdr + comp

            if len(packet) > MAX_UDP_SIZE:
                self.get_logger().warn(f"패킷 크기 초과 ({len(packet)}/{MAX_UDP_SIZE}), 생략")
                return

            self.sock.sendto(packet, (self.server_ip, self.server_port))
            self.get_logger().debug(f"전송: 압축된 {comp_len} 바이트")

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
    # 0.1초마다 송신 (필요시 조정)
    node.create_timer(0.1, node.timer_callback)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
