#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_interfaces.msg import BatteryStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import datetime

# DB 관련 import
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.models.roscars_models import RosCars

class RoscarReceiver(Node):
    def __init__(self):
        super().__init__('roscar_receiver')
        # DB 세션 준비
        self.dbm = DatabaseManager()
        self.session = self.dbm.get_session('roscars')

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        # SSID 토픽 구독
        self.ssid = None
        self.create_subscription(String, '/roscar/ssid',
                                 self.ssid_callback, qos.depth)

        # 배터리 토픽 구독
        self.create_subscription(BatteryStatus, '/roscar/status/battery',
                                 self.battery_callback, qos)

        self.get_logger().info('RoscarReceiver initialized.')

    def ssid_callback(self, msg: String):
        self.ssid = msg.data
        self.get_logger().info(f'[SSID] {self.ssid}')

    def battery_callback(self, msg: BatteryStatus):
        if not self.ssid:
            self.get_logger().warn("Battery msg before SSID, skipping")
            return

        percent = int(msg.battery_percent)
        ts = datetime.datetime.fromtimestamp(
            msg.stamp.sec + msg.stamp.nanosec * 1e-9
        )

        try:
            # 1) 해당 로봇 레코드 조회
            record = (self.session
                      .query(RosCars)
                      .filter_by(roscar_namespace=self.ssid)
                      .first())

            if record:
                # 2) 배터리 값이 변했을 때만 갱신
                if record.battery_percentage != percent:
                    old = record.battery_percentage
                    record.battery_percentage = percent
                    self.session.commit()
                    self.get_logger().info(
                        f"DB UPDATE [{self.ssid}]: Battery {old}% → {percent}% at {ts}")
                else:
                    self.get_logger().debug(
                        f"No change for [{self.ssid}], still {percent}%")
            else:
                # 3) 해당 SSID 로우가 없으면 삽입 (최초 한 번만)
                new_row = RosCars(
                    roscar_namespace=self.ssid,
                    battery_percentage=percent,
                    # 필수 컬럼은 채워주세요!
                )
                self.session.add(new_row)
                self.session.commit()
                self.get_logger().info(
                    f"DB INSERT new RosCars row for SSID={self.ssid} with {percent}%")

        except Exception as e:
            self.get_logger().error(f"DB error: {e}")
            self.session.rollback()

def main(args=None):
    rclpy.init(args=args)
    node = RoscarReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
