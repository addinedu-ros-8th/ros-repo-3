import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.utilities import remove_ros_args

from shared_interfaces.msg import BatteryStatus


qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


class BatteryReader(Node):
    def __init__(self, namespace: str):
        super().__init__('battery_reader')
        topic = f'/{namespace}/roscar/status/battery'

        self.subscription = self.create_subscription(
            BatteryStatus,
            topic,
            self.callback,
            qos_profile
        )
        self.get_logger().info(f"✅ Subscribed to topic: {topic}")

    def callback(self, msg):
        self.get_logger().info(f"[Battery Info] {msg.roscar_name} → {msg.percentage:.2f}%")


def main(args=None):
    rclpy.init(args=args)

    # ROS argument 제거 후 사용자 인자 추출
    user_args = remove_ros_args(args if args else sys.argv)
    if len(user_args) < 2:
        print("❗ 사용법: ros2 run <your_package_name> battery_reader <namespace>")
        rclpy.shutdown()
        return

    namespace = user_args[1]

    node = BatteryReader(namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
