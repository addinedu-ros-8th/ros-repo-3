from shared_interfaces.msg import RoscarRegister
import rclpy
from rclpy.node import Node

class RobotRegisterRequester(Node):
    def __init__(self):
        super().__init__('roscar_requester')
        self.subscription = self.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.roscar_status_callback,
            10
        )

    def roscar_status_callback(self, msg):
        # 로봇 등록 요청을 수신했을 때의 로그 출력
        self.get_logger().info(f"[수신] 로봇 등록 요청 수신됨: {msg.roscar_name}, IP: {msg.roscar_ip_v4},{msg.from_domain_id},  {msg.to_domain_id}")

        # GUI 알림
        self.notify_gui_roscar_added(msg)

    def notify_gui_roscar_added(self, msg):
        data = {
            "type": "RobotAddedNotification",
            "roscar_name": msg.roscar_name,
            "battery_percentage": msg.battery_percentage,
            "operational_status": msg.operational_status,
            "roscar_ip_v4": msg.roscar_ip_v4
        }
        # GUI에 전송할 데이터 출력
        self.get_logger().info(f"[GUI 알림] {data}")
