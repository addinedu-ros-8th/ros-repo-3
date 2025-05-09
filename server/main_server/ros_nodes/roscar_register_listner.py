from shared_interfaces.msg import RobotRegister
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class RobotRegisterRequester(Node):
    def __init__(self):
        super().__init__('roscar_requester')
        self.subscription = self.create_subscription(
            RobotRegister,
            '/roscar/register',
            self.roscar_status_callback,
            10
        )

    def roscar_status_callback(self, msg):
        # DB에서 기존 로봇 정보 조회
        existing_roscar = get_roscar_info(msg.roscar_name)

        if existing_roscar:
            existing_ip = existing_roscar['roscar_ip_v4']
            if existing_ip != msg.roscar_ip_v4:
                # IP 주소가 바뀐 경우 → 업데이트
                update_roscar_ip(msg.roscar_name, msg.roscar_ip_v4)
                self.get_logger().info(f"[업데이트] {msg.roscar_name} IP 변경됨: {existing_ip} → {msg.roscar_ip_v4}")
            else:
                self.get_logger().info(f"[중복] 등록된 로봇이며 IP 동일: {msg.roscar_name}")
            return

        # 신규 로봇 → 삽입
        insert_roscar(
            msg.roscar_name,
            msg.battery_percentage,
            msg.operational_status,
            msg.roscar_ip_v4
        )
        self.get_logger().info(f"[등록 완료] 새로운 로봇: {msg.roscar_name}")
        self.notify_gui_roscar_added(msg)

    def notify_gui_roscar_added(self, msg):
        data = {
            "type": "RobotAddedNotification",
            "roscar_name": msg.roscar_name,
            "battery_percentage": msg.battery_percentage,
            "operational_status": msg.operational_status,
            "roscar_ip_v4": msg.roscar_ip_v4
        }
        # TCP 통해 GUI에 전송
        # self.send_message_to_gui(data)
