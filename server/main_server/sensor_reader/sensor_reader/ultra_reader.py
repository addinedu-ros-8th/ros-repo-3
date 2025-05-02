import rclpy as rp
from rclpy.node import Node
from shared_interfaces.msg import UltraStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


class SLLidarClient(Node):
    def __init__(self):
        super().__init__('ultra_reader')

        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
        )
        self.subscription = self.create_subscription(
            UltraStatus,  # 받은 메시지 타입 지정
            '/robot/sensor/ultra',    # 구독할 토픽 이름
            self.callback,  # 콜백 함수
            qos_profile  # 큐 사이즈
        )

    def callback(self, msg):
        print(msg.roscar_name)
        print(msg.distance)

def main(args=None):
    rp.init(args=args)
    node = SLLidarClient()
    rp.spin(node)
    rp.shutdown()

if __name__ == '__main__':
    main()
