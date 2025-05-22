import rclpy
from rclpy.node import Node
from shared_interfaces.msg import ObstacleDetected

class ObstacleDetectedSubscriber(Node):
    def __init__(self):
        super().__init__('obstacle_detected_subscriber')
        self.sub = self.create_subscription(
            ObstacleDetected,
            '/roscar/obstacle/detected',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(
            f'수신: Detected id={msg.roscar_id} dist={msg.distance:.2f} dir={msg.direction:.2f} stamp={msg.stamp.sec}.{msg.stamp.nanosec}'
        )

def main():
    rclpy.init()
    node = ObstacleDetectedSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()