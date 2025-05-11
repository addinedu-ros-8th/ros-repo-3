import rclpy as rp
from rclpy.node import Node
from shared_interfaces.msg import UltraStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
from rclpy.utilities import remove_ros_args

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


class UltraReader(Node):
    def __init__(self, namespace: str):
        super().__init__('ultra_reader')

        # 동적으로 토픽을 구성
        topic = f'/{namespace}/roscar/sensor/ultra'

        self.subscription = self.create_subscription(
            UltraStatus,  # 받은 메시지 타입 지정
            topic,  # 구독할 토픽 이름
            self.callback,  # 콜백 함수
            qos_profile  # 큐 사이즈
        )

        self.get_logger().info(f"✅ Subscribed to topic: {topic}")

    def callback(self, msg):
        self.get_logger().info(f"[UltraSensor] {msg.roscar_name} → Distance: {msg.distance:.2f} m")


def main(args=None):
    rp.init(args=args)

    # ROS argument 제거 후 사용자 인자 추출
    user_args = remove_ros_args(args if args else sys.argv)
    if len(user_args) < 2:
        print("❗ 사용법: ros2 run <your_package_name> ultra_reader <namespace>")
        rp.shutdown()
        return

    namespace = user_args[1]

    node = UltraReader(namespace)
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
