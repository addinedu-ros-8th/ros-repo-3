#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from shared_interfaces.msg import BatteryStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import datetime

class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_status_subscriber')
        # 고정 토픽 이름
        topic = '/roscar/status/battery'

        # QoS: Reliable, keep last 10 messages
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        self.subscription = self.create_subscription(
            BatteryStatus,
            topic,
            self.listener_callback,
            qos
        )
        # prevent unused variable warning
        self.subscription  
        self.get_logger().info(f"Subscribed to topic: {topic}")

    def listener_callback(self, msg: BatteryStatus):
        # ROS time → Python datetime
        timestamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        ts = datetime.datetime.fromtimestamp(timestamp)

        # SSID 필드 없이 배터리 상태만 출력
        self.get_logger().info(
            f"Battery: {msg.battery_percent:.1f}% | "
            f"Charging: {msg.is_charging} | "
            f"Time: {ts.strftime('%Y-%m-%d %H:%M:%S')}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BatterySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
