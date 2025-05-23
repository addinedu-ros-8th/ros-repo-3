import rclpy
from rclpy.node import Node
from shared_interfaces.msg import ObstacleResponse

class ObstacleResponsePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_response_publisher')
        self.pub = self.create_publisher(ObstacleResponse, '/roscar/obstacle/response', 10)
        self.timer = self.create_timer(1.0, self.publish_response)

    def publish_response(self):
        msg = ObstacleResponse()
        msg.roscar_id = 1
        msg.command = 0x01  # 예: 회피
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f'발행: ObstacleResponse {msg}')

def main():
    rclpy.init()
    node = ObstacleResponsePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()