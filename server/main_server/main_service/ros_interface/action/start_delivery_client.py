import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from shared_interfaces.action import StartDelivery
from shared_interfaces.msg import StartTask


class StartDeliveryClient(Node):
    def __init__(self):
        super().__init__('start_delivery_client')
        self._client = ActionClient(self, StartDelivery, '/pinky_07db/start_delivery')  # 예시 namespace 사용

    def send_goal(self, delivery_id: int, tasks: list[dict]):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('[StartDeliveryClient] Action server not available')
            return

        goal_msg = StartDelivery.Goal()
        goal_msg.delivery_id = delivery_id
        goal_msg.tasks = []

        for task in tasks:
            task_msg = StartTask()
            task_msg.task_id = task["task_id"]
            task_msg.shoes_model_id = task["shoes_model_id"]
            task_msg.location_id = task["location_id"]
            goal_msg.tasks.append(task_msg)

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('[StartDeliveryClient] Goal rejected')
            return

        self.get_logger().info('[StartDeliveryClient] Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[StartDeliveryClient] 피드백: {feedback.phase}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'[StartDeliveryClient] 결과: success={result.success}')


def main(args=None):
    rclpy.init(args=args)
    node = StartDeliveryClient()

    # 테스트용 예시 요청
    node.send_goal(
        delivery_id=1234,
        tasks=[
            {"task_id": 1, "shoes_model_id": 101, "location_id": 501},
            {"task_id": 2, "shoes_model_id": 102, "location_id": 502}
        ]
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
