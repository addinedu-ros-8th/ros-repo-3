import rclpy as rp
from rclpy.node import Node
from shared_interfaces.msg import LidarScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


class SLLidarClient(Node):
    def __init__(self):
        super().__init__('sllidar_client')

        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
        )
        self.subscription = self.create_subscription(
            LidarScan,  # 받은 메시지 타입 지정
            '/scan',    # 구독할 토픽 이름
            self.callback,  # 콜백 함수
            qos_profile  # 큐 사이즈
        )

    def callback(self, msg):
        for i, range_value in enumerate(msg.ranges):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            # 음수 각도를 0~360 범위로 변환
            angle_deg = angle_deg % 360

            if range_value != float('inf'):
                self.get_logger().info(f"Angle {angle_deg:.2f}°: {range_value:.2f} m")

def main(args=None):
    rp.init(args=args)
    node = SLLidarClient()
    rp.spin(node)
    rp.shutdown()

if __name__ == '__main__':
    main()
