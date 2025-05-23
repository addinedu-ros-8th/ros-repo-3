#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from shared_interfaces.action import MaintenanceCharge

class MaintenanceChargeActionClient(Node):

    def __init__(self):
        super().__init__('maintenance_charge_action_client')
        self._client = ActionClient(self, MaintenanceCharge, '/maintenance/charge')
        self.send_goal()

    def send_goal(self):
        self._client.wait_for_server()
        goal_msg = MaintenanceCharge.Goal()
        goal_msg.roscar_id = 1
        goal_msg.charge_zone = "DockA"

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'단계={fb.phase}, 배터리={fb.battery_percent:.2f}%')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'충전 완료: success={result.success}, message="{result.message}"')
        rclpy.shutdown()

def main():
    rclpy.init()
    MaintenanceChargeActionClient()
    rclpy.spin()

if __name__ == '__main__':
    main()
