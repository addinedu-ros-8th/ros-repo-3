import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from shared_interfaces.action import SecurityPatrol

class SecurityPatrolActionClient(Node):

    def __init__(self):
        super().__init__('security_patrol_action_client')
        self._client = ActionClient(self, SecurityPatrol, '/security/patrol')
        self.send_goal()

    def send_goal(self):
        self._client.wait_for_server()
        goal_msg = SecurityPatrol.Goal()
        goal_msg.roscar_id = 1
        goal_msg.patrol_zone = "전체"

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'순찰 중: zone={fb.current_zone}, event={fb.event}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'순찰 완료: success={result.success}, message="{result.message}"')
        rclpy.shutdown()

def main():
    rclpy.init()
    SecurityPatrolActionClient()
    rclpy.spin()

if __name__ == '__main__':
    main()