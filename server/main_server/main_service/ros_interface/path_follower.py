#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.map_ready = False

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, qos)
        self.cli = self.create_client(GetPlan, '/compute_path')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)

        self.pub = self.create_publisher(Path, '/global_path', 10)
        self.timer = self.create_timer(0.5, self._try_call)

        self.get_logger().info('PathFollower initialized…')

    def _map_cb(self, msg):
        if not self.map_ready:
            self.map_ready = True
            self.get_logger().info('PathFollower: Map received')

    def _try_call(self):
        if not self.map_ready:
            return
        if not self.cli.wait_for_service(timeout_sec=1.0):
            return

        req = GetPlan.Request()
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.get_parameter('start_x').value
        start.pose.position.y = self.get_parameter('start_y').value
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.get_parameter('goal_x').value
        goal.pose.position.y = self.get_parameter('goal_y').value
        goal.pose.orientation.w = 1.0

        req.start = start
        req.goal  = goal
        req.tolerance = 0.0

        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)
        self.timer.cancel()

    def _on_response(self, future):
        res = future.result()
        self.get_logger().info(f'Received path with {len(res.plan.poses)} points')
        self.pub.publish(res.plan)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
