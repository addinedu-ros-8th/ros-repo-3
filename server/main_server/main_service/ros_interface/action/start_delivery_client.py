import sys
import os
import json
import rclpy

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from shared_interfaces.action import StartDelivery
from shared_interfaces.msg import StartTask

class StartDeliveryClient(Node):
    def __init__(self, namespace: str):
        super().__init__(f'start_delivery_client_{namespace}', namespace=f"/{namespace}")
        action_name = f"/{namespace}/start_delivery"
        self._client = ActionClient(self, StartDelivery, action_name)

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


def main():
    print("[CLIENT] main() 시작됨")
    
    if len(sys.argv) != 4:
        print("Usage: python start_delivery_client.py <namespace> <domain_id> <json_path>")
        sys.exit(1)

    namespace   = sys.argv[1]
    domain_id   = sys.argv[2]
    json_path   = sys.argv[3]

    os.environ['ROS_DOMAIN_ID'] = domain_id
    rclpy.init()

    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"[❌ JSON 파싱 실패] {e}")
        sys.exit(1)


    delivery_id = data['delivery_id']
    tasks = data['tasks']

    node = StartDeliveryClient(namespace)
    node.send_goal(delivery_id=delivery_id, tasks=tasks)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
