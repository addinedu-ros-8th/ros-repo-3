import rclpy as rp
from rclpy.node import Node
from shared_interfaces.msg import BatteryStatus
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
            BatteryStatus,  # 받은 메시지 타입 지정
            '/roscar/status/battery',    # 구독할 토픽 이름
            self.callback,  # 콜백 함수
            qos_profile  # 큐 사이즈
        )

    def callback(self, msg):
        print(msg.roscar_name)
        print(msg.percentage)

def main(args=None):
    rp.init(args=args)
    node = SLLidarClient()
    rp.spin(node)
    rp.shutdown()

if __name__ == '__main__':
    main()
