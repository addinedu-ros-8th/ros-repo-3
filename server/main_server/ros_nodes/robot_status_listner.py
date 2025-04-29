from shared_interfaces.msg import RobotStatus
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class RobotStatusRequester(Node):
    def __init__(self):
        super().__init__('robot_status_requester')
        self.publisher_ = self.create_publisher(String, '/robot_status_request', 10)
        self.subscription = self.create_subscription(
            RobotStatus,
            '/robot_status_response',
            self.robot_status_callback,
            10
        )

    def request_robot_status(self, roscar_ip_v4):
        msg = String()
        msg.data = roscar_ip_v4
        self.publisher_.publish(msg)
        self.get_logger().info(f'Requested status for {roscar_ip_v4}')

    def robot_status_callback(self, msg):
        # DB에 저장
        insert_roscar(
            msg.roscar_id,
            msg.roscar_name,
            msg.battery_percentage,
            msg.operational_status,
            msg.roscar_ip_v4
        )
        # GUI 업데이트
        self.notify_gui_robot_added(msg)
    
    def notify_gui_robot_added(self, msg):
        data = {
            "type": "RobotAddedNotification",
            "roscar_id": msg.roscar_id,
            "roscar_name": msg.roscar_name,
            "battery_percentage": msg.battery_percentage,
            "operational_status": msg.operational_status,
            "roscar_ip_v4": msg.roscar_ip_v4
        }
        # TCP 통해 GUI에 전송 (send_message_to_gui 메서드 필요)
        self.send_message_to_gui(data)
