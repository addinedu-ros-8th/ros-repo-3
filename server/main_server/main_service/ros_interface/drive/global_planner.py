#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetPlan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from ros_interface.a_star_planner import AStarPlanner

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        qos = rclpy.qos.QoSProfile(depth=1)
        qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        self.grid    = None
        self.planner = None
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.srv = self.create_service(GetPlan, '/compute_path', self.handle_compute_path)
        self.get_logger().info('GlobalPlannerNode ready')

    def map_callback(self, msg):
        self.grid    = msg
        self.planner = AStarPlanner(msg)
        self.get_logger().info('Map 수신 완료')

    def handle_compute_path(self, req, res):
        if not self.planner:
            self.get_logger().warn('Map not ready')
            return res
        poses = self.planner.compute_path(req.start.pose, req.goal.pose)
        res.plan.poses = poses
        res.plan.header.frame_id = self.grid.header.frame_id
        res.plan.header.stamp = self.get_clock().now().to_msg()
        return res

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    return 0

if __name__=='__main__':
    sys.exit(main())
