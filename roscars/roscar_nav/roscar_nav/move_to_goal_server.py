#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from shared_interfaces.action import MoveToGoal
import time


class MoveToGoalServer(Node):
    def __init__(self):
        super().__init__('move_to_goal_server')

        self._action_server = ActionServer(
            self,
            MoveToGoal,
            'move_to_goal',
            self.execute_callback
        )

        self.get_logger().info("✅ MoveToGoal Action Server is up.")

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info(
            f"Received goal: roscar_id={request.roscar_id}, "
            f"x={request.goal_x}, y={request.goal_y}, theta={request.theta}"
        )

        feedback = MoveToGoal.Feedback()
        for i in range(0, 101, 20):
            feedback.progress = i
            feedback.current_x = request.goal_x * (i / 100.0)
            feedback.current_y = request.goal_y * (i / 100.0)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        result = MoveToGoal.Result()
        result.success = True
        result.message = "Arrived at destination (simulated)"
        self.get_logger().info("✅ Goal processing complete")
        return result


def main():
    rclpy.init()
    node = MoveToGoalServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
