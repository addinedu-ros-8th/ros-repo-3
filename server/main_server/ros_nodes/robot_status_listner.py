# server/main_server/ros_nodes/robot_status_listener.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotStatusListener(Node):
    def __init__(self):
        super().__init__('robot_status_listener')

        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.listener_callback,
            10  # QoS profile (queue depth)
        )

        self.get_logger().info('RobotStatusListener node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received robot status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
