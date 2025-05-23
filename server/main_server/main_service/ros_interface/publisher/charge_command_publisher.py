import rclpy
from rclpy.node import Node
from shared_interfaces.msg import ChargeCommand

class ChargeCommandPublisher(Node):
    def __init__(self):
        super().__init__('charge_command_publisher')
        self.pub = self.create_publisher(ChargeCommand, '/roscar/status/charge', 10)
        self.timer = self.create_timer(1.0, self.publish_command)

    def publish_command(self):
        msg = ChargeCommand()
        msg.roscar_id = 1
        msg.goal_x = 1.0
        msg.goal_y = 2.0
        msg.theta = 0.0
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f'발행: ChargeCommand {msg}')

def main():
    rclpy.init()
    node = ChargeCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
