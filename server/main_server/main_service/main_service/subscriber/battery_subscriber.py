#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from shared_interfaces.msg import BatteryStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import datetime


def get_ap_ssid():
    """
    /etc/hostapd/hostapd.conf 파일에서 ssid 값을 읽어옵니다.
    파일을 읽을 수 없거나 ssid 값이 없으면 'UNKNOWN_SSID'를 반환합니다.
    """
    try:
        with open('/etc/hostapd/hostapd.conf', 'r') as f:
            for line in f:
                if line.startswith('ssid='):
                    return line.strip()[5:]
    except Exception:
        pass
    return 'UNKNOWN_SSID'


class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_status_subscriber')
        ssid = get_ap_ssid()
        topic = f"/{ssid}/roscar/status/battery"

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
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f"Subscribed to topic: {topic}")

    def listener_callback(self, msg: BatteryStatus):
        # Convert ROS time to Python datetime
        timestamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        ts = datetime.datetime.fromtimestamp(timestamp)

        self.get_logger().info(
            f"[{msg.roscar_namespace}] Battery: {msg.battery_percent:.1f}% | "
            f"Charging: {msg.is_charging} | Time: {ts.strftime('%Y-%m-%d %H:%M:%S')}"
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
