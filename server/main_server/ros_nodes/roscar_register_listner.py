import rclpy
from rclpy.node import Node
from shared_interfaces.msg import RoscarRegister

class RoscarRegisterListener(Node):
    def __init__(self):
        super().__init__('roscar_register_listener')
        
        # 구독할 토픽 이름
        self.subscription = self.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.listener_callback,
            10  # 큐 사이즈
        )
        
    def listener_callback(self, msg):
        self.get_logger().info(f"Received Roscar Register: {msg.register_data}")

def main(args=None):
    rclpy.init(args=args)
    node = RoscarRegisterListener()
    rclpy.spin(node)  # 노드를 계속 실행 상태로 유지
    rclpy.shutdown()

if __name__ == '__main__':
    main()
