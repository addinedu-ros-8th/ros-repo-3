from shared_interfaces.msg import RobotStatus
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class RobotStatusResponder(Node):
    def __init__(self):
        super().__init__('robot_status_responder')
        self.subscription = self.create_subscription(
            String,
            '/robot_status_request',
            self.handle_request,
            10
        )
        self.publisher_ = self.create_publisher(RobotStatus, '/robot_status_response', 10)

    def handle_request(self, msg):
        requested_ip = msg.data
        self.get_logger().info(f'Received status request for {requested_ip}')
        
        # 예시: 로봇 스스로 상태를 채워야 함
        response = RobotStatus()
        response.roscar_id = 1
        response.roscar_name = "robot_001"
        response.battery_percentage = 90
        response.operational_status = "ACTIVE"
        response.roscar_ip_v4 = requested_ip

        self.publisher_.publish(response)
        self.get_logger().info(f'Published robot status for {requested_ip}')
