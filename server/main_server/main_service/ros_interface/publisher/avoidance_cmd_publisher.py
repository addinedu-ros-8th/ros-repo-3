import rclpy
from rclpy.node import Node
from shared_interfaces.msg import ObstacleAvoidanceCmd

class AvoidanceCmdPublisher(Node):
    def __init__(self):
        super().__init__('avoidance_cmd_publisher')
        self.pub = self.create_publisher(ObstacleAvoidanceCmd, '/roscar/avoidance/cmd', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd)

    def publish_cmd(self):
        msg = ObstacleAvoidanceCmd()
        msg.roscar_id = 1
        msg.direction = 90.0
        msg.speed = 0.5
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f'발행: ObstacleAvoidanceCmd {msg}')

def main():
    rclpy.init()
    node = AvoidanceCmdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()