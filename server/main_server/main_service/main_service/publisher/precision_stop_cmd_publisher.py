import rclpy
from rclpy.node import Node
from shared_interfaces.msg import PrecisionStopCmd

class PrecisionStopCmdPublisher(Node):
    def __init__(self):
        super().__init__('precision_stop_cmd_publisher')
        self.pub = self.create_publisher(PrecisionStopCmd, '/roscar/precision_stop/cmd', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd)

    def publish_cmd(self):
        msg = PrecisionStopCmd()
        msg.roscar_id = 1
        msg.target_x = 2.0
        msg.target_y = 2.0
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f'발행: PrecisionStopCmd {msg}')

def main():
    rclpy.init()
    node = PrecisionStopCmdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()