import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from shared_interfaces.action import ScanInventory

class ScanInventoryActionClient(Node):

    def __init__(self):
        super().__init__('scan_inventory_action_client')
        self._client = ActionClient(self, ScanInventory, '/inventory/scan_items')
        self.send_goal()

    def send_goal(self):
        self._client.wait_for_server()
        goal_msg = ScanInventory.Goal()
        goal_msg.scan_zone = "ZoneA"
        goal_msg.roscar_id = 1

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'스캔 중: zone={fb.zone}, count={fb.scanned_item}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'스캔 완료: total={result.total_items}, success={result.success}')
        rclpy.shutdown()

def main():
    rclpy.init()
    ScanInventoryActionClient()
    rclpy.spin()

if __name__ == '__main__':
    main()