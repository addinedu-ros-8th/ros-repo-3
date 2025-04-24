import rclpy
from rclpy.node import Node
from shared_interfaces.msg import TaskResult
class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.publisher_ = self.create_publisher(TaskResult, 'task_results', 10)
        self.timer = self.create_timer(5.0, self.publish_dummy_result)
        self.get_logger().info('TaskManager initialized')
    def publish_dummy_result(self):
        msg = TaskResult()
        msg.task_id = "dummy"
        msg.result = "success"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published dummy task result: {msg.task_id} - {msg.result}')
def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()