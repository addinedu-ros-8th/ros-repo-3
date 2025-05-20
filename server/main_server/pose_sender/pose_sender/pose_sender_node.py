#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from shared_interfaces.msg import RoscarRegister

class MainServerBridge(Node):
    def __init__(self):
        super().__init__('main_server_bridge')

        # 실제로 발행되는 토픽 구독
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        # RoscarRegister를 직접 발행하는 노드를 로봇 쪽에 띄워야 함
        self.create_subscription(RoscarRegister, '/roscar/register', self.reg_cb, 10)

        # GUI용 재발행 토픽
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/monitor/pose', 10)
        self.map_pub  = self.create_publisher(
            OccupancyGrid,               '/monitor/map',  10)
        self.reg_pub  = self.create_publisher(
            RoscarRegister,              '/monitor/register', 10)

    def odom_cb(self, odom: Odometry):
        # Odometry → PoseWithCovarianceStamped 변환
        p = PoseWithCovarianceStamped()
        p.header = odom.header
        p.pose = odom.pose
        self.pose_pub.publish(p)

    def map_cb(self, msg: OccupancyGrid):
        self.map_pub.publish(msg)

    def reg_cb(self, msg: RoscarRegister):
        self.reg_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MainServerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
