#!/usr/bin/env python3
import sys
import os
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from shared_interfaces.action import StartDelivery
from shared_interfaces.msg import StartTask

class StartDeliveryClient(Node):
    def __init__(self, namespace: str):
        # Node 이름 및 네임스페이스 설정
        super().__init__(f'start_delivery_client_{namespace}', namespace=f"/{namespace}")
        # goal_position.json 로드 및 매핑 생성
        try:
            share_dir = get_package_share_directory('main_service')
            config_path = os.path.join(share_dir, 'config', 'goal_position.json')
            with open(config_path, 'r') as f:
                entries = json.load(f)
            # 리스트 → dict 매핑: {"S1": {x, y, theta}, ...}
            self.goal_map = {
                e['goal_position']: {'x': e['x'], 'y': e['y'], 'theta': e['theta']}
                for e in entries
            }
            self.get_logger().info(f"[StartDeliveryClient] Loaded {len(self.goal_map)} goal positions")
        except Exception as e:
            self.get_logger().error(f"[StartDeliveryClient] Failed to load goal_position.json: {e}")
            self.goal_map = {}

        # ActionClient 초기화
        action_name = f"/{namespace}/start_delivery"
        self._client = ActionClient(self, StartDelivery, action_name)

    def send_goal(self, delivery_id: int, tasks: list[dict]):
        # 액션 서버 준비 확인
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('[StartDeliveryClient] Action server not available')
            return

        # Goal 메시지 구성
        goal_msg = StartDelivery.Goal()
        goal_msg.delivery_id = delivery_id
        goal_msg.tasks = []

        for t in tasks:
            task_msg = StartTask()
            # location(GoalPosition) → 좌표 매핑
            if 'goal_position' in t:
                pos = t['goal_position']
                mapping = self.goal_map.get(pos)
                if mapping:
                    task_msg.location = pos
                    task_msg.x = float(mapping['x'])
                    task_msg.y = float(mapping['y'])
                    task_msg.theta = float(mapping['theta'])
                else:
                    self.get_logger().warn(f"[StartDeliveryClient] Unknown goal_position: {pos}")
            # shoes_model_id
            if 'shoes_model_id' in t:
                task_msg.shoes_model_id = int(t['shoes_model_id'])
            # quantity
            if 'quantity' in t:
                task_msg.quantity = int(t['quantity'])

            goal_msg.tasks.append(task_msg)

        self.get_logger().info(f"[StartDeliveryClient] Sending delivery_id={delivery_id}, tasks_count={len(goal_msg.tasks)}")
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('[StartDeliveryClient] Goal rejected')
            return
        self.get_logger().info('[StartDeliveryClient] Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[StartDeliveryClient] Feedback: phase={feedback.phase}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'[StartDeliveryClient] Result: success={result.success}')


def main():
    print("[CLIENT] main() 시작됨")

    if len(sys.argv) != 4:
        print("Usage: python start_delivery_client.py <namespace> <domain_id> <json_path>")
        sys.exit(1)

    namespace = sys.argv[1]
    domain_id = sys.argv[2]
    json_path = sys.argv[3]

    # ROS_DOMAIN_ID 설정
    os.environ['ROS_DOMAIN_ID'] = domain_id
    rclpy.init()

    # JSON 파일 로드
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"[❌ JSON 파싱 실패] {e}")
        sys.exit(1)

    delivery_id = int(data.get('delivery_id', 0))
    tasks = data.get('tasks', [])

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