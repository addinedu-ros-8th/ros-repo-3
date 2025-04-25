import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from shared_interfaces.srv import ManualOverride
from shared_interfaces.msg import BatteryStatus
from pinkylib.battery import Battery  # 실제 배터리 클래스

import threading
import time


class ManualOverrideClient(Node):
    def __init__(self):
        super().__init__('controller_node')

        # QoS 설정 (신뢰성 보장)
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 서비스 클라이언트 생성
        self.cli = self.create_client(ManualOverride, 'manual_override')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for service "manual_override"...')
        self.get_logger().info('✅ Connected to manual_override service')

        # 배터리 퍼블리셔 설정
        self.publisher = self.create_publisher(BatteryStatus, '/robot/status/battery', qos_profile)

        self.robot_id = 1  # 현재 로봇 ID
        self.battery_sensor = Battery()  # 배터리 센서 클래스 초기화

        # 배터리 송신 쓰레드 시작
        self._battery_thread = threading.Thread(target=self.publish_battery_status_loop, daemon=True)
        self._battery_thread.start()

    def send_override_request(self, robot_id, command_text):
        req = ManualOverride.Request()
        req.robot_id = robot_id
        req.command = command_text
        self.future = self.cli.call_async(req)

    def publish_battery_status_loop(self):
        while rclpy.ok():
            try:
                percent = self.battery_sensor.get_battery()
                msg = BatteryStatus()
                msg.robot_id = self.robot_id
                msg.battery_percent = percent
                self.publisher.publish(msg)
                self.get_logger().info(f'🔋 Battery status published: {percent:.1f}%')
                time.sleep(1.0)  # 1초 주기
            except Exception as e:
                self.get_logger().error(f"⚠️ Battery read error: {e}")
                time.sleep(2.0)

    def destroy_node(self):
        try:
            self.battery_sensor.clean()  # I2C 정리
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualOverrideClient()

    # 수동 오버라이드 명령 (예시)
    node.send_override_request(robot_id=1, command_text="STOP")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
