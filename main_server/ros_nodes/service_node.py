import rclpy
from rclpy.node import Node
from shared_interfaces.srv import ManualOverride
class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(ManualOverride, 'manual_override', self.handle_override)
        self.get_logger().info('ServiceNode started and ready to receive override requests')
    def handle_override(self, request, response):
        self.get_logger().info(f'Received manual override: {request.command}')
        # TODO: Add logic to handle manual override request
        response.success = True
        return response
def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()