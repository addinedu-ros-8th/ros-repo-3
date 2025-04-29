import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import os

class SequentialNavigator(Node):
    def __init__(self):
        super().__init__('sequential_navigator')

        # NavigateToPose 액션 클라이언트 생성
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Waypoints 불러오기
        self.waypoints = self.load_waypoints()
        self.current_idx = 0

        # 액션 서버 연결 후 주기적으로 이동 시작
        self._timer = self.create_timer(1.0, self.navigate_next)

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
                pose.pose.orientation.w = 1.0  # 기본 방향
                waypoints.append(pose)
        return waypoints

    def navigate_next(self):
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for navigate_to_pose action server...')
            return

        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            self._timer.cancel()
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.waypoints[self.current_idx]
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Navigating to waypoint {self.current_idx + 1}/{len(self.waypoints)}...')

        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # 타이머 멈춰서 중복 이동 방지
        self._timer.cancel()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code}')

        self.current_idx += 1
        # 다음 Waypoint로 이동 재시작
        self._timer = self.create_timer(1.0, self.navigate_next)


def main(args=None):
    rclpy.init(args=args)
    node = SequentialNavigator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
