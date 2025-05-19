#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from shared_interfaces.action import MoveToGoal

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.get_logger().info("GoalSender Node Started")

def main():
    rclpy.init()
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
