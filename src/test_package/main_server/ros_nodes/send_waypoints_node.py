import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import os

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        # FollowWaypoints 액션 클라이언트 생성
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Waypoints 불러오기
        self.waypoints = self.load_waypoints()

        # 액션 서버 연결 후 전송 시작
        self._client.wait_for_server()
        self.send_waypoints()

    def load_waypoints(self):
        waypoints = []
        file_path = os.path.expanduser('~/saved_a_star_path.txt')
        with open(file_path, 'r') as f:
            for line in f:
                x, y = map(float, line.strip().split(','))
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0  # 기본 방향 (회전 없음)
                waypoints.append(pose)
        return waypoints

    def send_waypoints(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        self.get_logger().info(f'Sending {len(self.waypoints)} waypoints to waypoint follower...')

        self._send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Currently at waypoint {feedback.current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowWaypoints.Result.SUCCESS:
            self.get_logger().info('Successfully followed all waypoints!')
        else:
            self.get_logger().error(f'Failed with error code: {result.error_code}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
