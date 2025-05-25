import rclpy
from rclpy.node import Node
from shared_interfaces.msg import PrecisionStopResult

class PrecisionStopResultSubscriber(Node):
    def __init__(self):
        super().__init__('precision_stop_result_subscriber')
        self.sub = self.create_subscription(
            PrecisionStopResult,
            '/roscar/precision_stop/result',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(
            f'수신: StopResult id={msg.roscar_id} success={msg.success} deviation={msg.deviation:.2f} stamp={msg.stamp.sec}.{msg.stamp.nanosec}'
        )

def main():
    rclpy.init()
    node = PrecisionStopResultSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()