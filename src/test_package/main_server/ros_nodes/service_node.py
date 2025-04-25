import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from shared_interfaces.srv import ManualOverride
from shared_interfaces.msg import BatteryStatus
from main_server.network.tcp_handler import TCPHandler

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # TCP 핸들러 초기화
        self.tcp = TCPHandler()

        # 서비스 서버 생성
        self.srv = self.create_service(ManualOverride, 'manual_override', self.handle_override)
        self.get_logger().info('✅ ServiceNode started and ready to receive override requests')

        # QoS 설정
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 배터리 상태 토픽 구독
        self.subscription = self.create_subscription(
            BatteryStatus,
            '/robot/status/battery',
            self.battery_callback,
            qos_profile=qos_profile
        )
        self.subscription  # prevent unused variable warning

    def handle_override(self, request, response):
        self.get_logger().info(f'🛠️ Received override from robot {request.robot_id}: {request.command}')
        # TODO: Add logic using request.robot_id
        response.success = True
        return response

    def battery_callback(self, msg: BatteryStatus):
        print("✅ Callback triggered!")
        self.get_logger().info(
            f'🔋 Battery from robot {msg.robot_id}: {msg.battery_percent:.1f}%'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
