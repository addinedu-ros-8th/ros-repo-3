import os
import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from shared_interfaces.action import MoveToGoal

# 패키지 공유 디렉터리 아래 config 폴더의 goal_position.json 경로
GOALS_JSON = os.path.join(
    get_package_share_directory('main_service'),
    'config',
    'goal_position.json'
)

class MoveToGoalClient(Node):
    def __init__(self):
        super().__init__('move_to_goal_client')
        self._client = ActionClient(self, MoveToGoal, 'move_to_goal')

        # JSON 파일 로드
        try:
            with open(GOALS_JSON, 'r') as f:
                self.goals = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load goals JSON: {e}')
            self.goals = []

    def send_goal(self, position_name: str):
        # JSON에서 해당 position 찾기
        entry = next((g for g in self.goals if g['goal_position'] == position_name), None)
        if entry is None:
            self.get_logger().error(f'Unknown position: {position_name}')
            return

        self._client.wait_for_server()
        goal = MoveToGoal.Goal()
        goal.roscar_id     = 1
        goal.goal_position = position_name          # position_name 포함
        goal.goal_x        = entry['x']
        goal.goal_y        = entry['y']
        goal.theta         = entry.get('theta', 0.0)

        self._client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.get_logger().info(
            f'Sent goal {position_name}: '
            f'({goal.goal_x:.3f}, {goal.goal_y:.3f}, θ={goal.theta:.3f})'
        )

    def feedback_callback(self, feedback):
        f = feedback.feedback
        self.get_logger().info(
            f'Progress: {f.progress}% at '
            f'({f.current_x:.3f}, {f.current_y:.3f})'
        )

def main(args=None):
    rclpy.init(args=args)
    client = MoveToGoalClient()
    client.send_goal('R1')  # 예: 'R1' 을 다른 위치명으로 바꿔 호출
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
