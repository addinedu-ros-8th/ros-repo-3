#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_interfaces.msg import BatteryStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import datetime

class RoscarReceiver(Node):
    def __init__(self):
        super().__init__('roscar_receiver')

        # QoS 동일 설정
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        # 1) SSID 토픽 구독
        self.ssid = None
        self.create_subscription(
            String,
            '/roscar/ssid',
            self.ssid_callback,
            qos.depth
        )

        # 2) 배터리 토픽 구독
        self.create_subscription(
            BatteryStatus,
            '/roscar/status/battery',
            self.battery_callback,
            qos
        )

        self.get_logger().info('RoscarReceiver initialized: subscribing to /roscar/ssid and /roscar/status/battery')

    def ssid_callback(self, msg: String):
        self.ssid = msg.data
        self.get_logger().info(f'[SSID] {self.ssid}')

    def battery_callback(self, msg: BatteryStatus):
        # SSID가 아직 없으면 "UNKNOWN"
        ssid = self.ssid or 'UNKNOWN'
        ts = datetime.datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec * 1e-9)
        self.get_logger().info(
            f"[{ssid}] Battery: {msg.battery_percent:.1f}% | "
            f"Charging: {msg.is_charging} | Time: {ts.strftime('%Y-%m-%d %H:%M:%S')}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RoscarReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested, exiting.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
