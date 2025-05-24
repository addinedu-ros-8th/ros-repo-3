#!/usr/bin/env python3
import datetime
from rclpy.node import Node
from std_msgs.msg import String
from shared_interfaces.msg import BatteryStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from server.main_server.databases.models.roscars_models import RosCars

class RoscarStatusSubscriber(Node):
    def __init__(self, main_ctl_service):
        super().__init__('roscar_status_subscriber')
        self.main_ctl_service = main_ctl_service

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        # SSID 토픽 구독
        self.ssid = None
        self._last_ssid = None   # 이전에 로그로 기록된 SSID
        self.create_subscription(
            String,
            '/roscar/ssid',
            self.ssid_callback,
            qos.depth
        )

        # 배터리 토픽 구독
        self.create_subscription(
            BatteryStatus,
            '/roscar/status/battery',
            self.battery_callback,
            qos
        )

        self.get_logger().info('RoscarReceiver initialized.')

    def ssid_callback(self, msg: String):
        # 이전과 다른 SSID일 때만 로그
        if msg.data != self._last_ssid:
            self._last_ssid = msg.data
            self.ssid = msg.data
            self.get_logger().info(f'[SSID] {self.ssid}')
        else:
            # 동일 SSID, 로그 생략
            pass
        
    def battery_callback(self, msg: BatteryStatus):
        if not self.ssid:
            self.get_logger().warn("Battery msg before SSID, skipping")
            return

        percent = int(msg.battery_percent)

        try:
            self.main_ctl_service.schema.update_roscar_status(
                namespace=self.ssid,
                battery=percent,
            )
            self.get_logger().info(f"Battery 상태 업데이트 완료: {self.ssid} = {percent}%")
        except Exception as e:
            self.get_logger().error(f"배터리 상태 업데이트 실패: {e}")
