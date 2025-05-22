import rclpy
from rclpy.node import Node
from shared_interfaces.msg import DashboardStatus

class DashboardStatusPublisher(Node):
    def __init__(self):
        super().__init__('dashboard_status_publisher')
        self.pub = self.create_publisher(DashboardStatus, '/dashboard/status/update', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = DashboardStatus()
        msg.roscar_id = 1
        msg.task_id = 42
        msg.pose_x = 0.5
        msg.pose_y = 0.5
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f'발행: DashboardStatus {msg}')

def main():
    rclpy.init()
    node = DashboardStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()