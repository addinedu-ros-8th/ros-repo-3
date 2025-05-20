import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from shared_interfaces.action import MoveToGoal

class MoveToGoalClient(Node):
    def __init__(self):
        super().__init__('move_to_goal_client')
        self._client = ActionClient(self, MoveToGoal, 'move_to_goal')

    def send_goal(self):
        self._client.wait_for_server()
        goal = MoveToGoal.Goal()
        goal.roscar_id = 1
        goal.goal_x = 2.0
        goal.goal_y = 3.0
        goal.theta = 1.57

        self._client.send_goal_async(goal, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback):
        f = feedback.feedback
        self.get_logger().info(f'Progress: {f.progress}% at ({f.current_x:.2f}, {f.current_y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    client = MoveToGoalClient()
    client.send_goal()
    rclpy.spin(client)
    rclpy.shutdown()
