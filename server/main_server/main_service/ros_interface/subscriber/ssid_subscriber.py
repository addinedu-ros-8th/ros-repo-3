#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SSIDSubscriber(Node):
    def __init__(self):
        super().__init__('ssid_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/roscar/ssid',
            self.ssid_callback,
            10  # QoS depth
        )
        self.subscription
        self.current_ssid = None
        self.get_logger().info('Subscribed to /roscar/ssid')

    def ssid_callback(self, msg: String):
        self.current_ssid = msg.data
        self.get_logger().info(f'Received SSID: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SSIDSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
