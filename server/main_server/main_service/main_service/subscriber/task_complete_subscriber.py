import rclpy
from rclpy.node import Node
from shared_interfaces.msg import TaskComplete

class TaskCompleteSubscriber(Node):
    def __init__(self):
        super().__init__('task_complete_subscriber')
        self.sub = self.create_subscription(
            TaskComplete,
            '/roscar/task/complete',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(
            f'수신: Complete id={msg.roscar_id} task={msg.task_id} success={msg.success} stamp={msg.stamp.sec}.{msg.stamp.nanosec}'
        )

def main():
    rclpy.init()
    node = TaskCompleteSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()