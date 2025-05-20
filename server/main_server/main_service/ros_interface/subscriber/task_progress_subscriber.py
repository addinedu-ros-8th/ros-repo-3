import rclpy
from rclpy.node import Node
from shared_interfaces.msg import TaskProgress

class TaskProgressSubscriber(Node):
    def __init__(self):
        super().__init__('task_progress_subscriber')
        self.sub = self.create_subscription(
            TaskProgress,
            '/roscar/task/progress',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(
            f'수신: Progress id={msg.roscar_id} task={msg.task_id} {msg.progress}% stamp={msg.stamp.sec}.{msg.stamp.nanosec}'
        )

def main():
    rclpy.init()
    node = TaskProgressSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()