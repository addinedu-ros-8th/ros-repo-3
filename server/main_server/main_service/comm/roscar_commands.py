import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shared_interfaces.action.MoveToGoal import MoveToGoal

class MoveToGoalClient(Node):
    """
    ROS2 ActionClient for MoveToGoal action. Use action_name to target specific server (pickup, storage, standby).
    """
    def __init__(self, action_name: str):
        super().__init__(f'{action_name}_client')
        self._action_name = action_name
        self._action_client = ActionClient(self, MoveToGoal, action_name)

    def send_goal(self, roscar_id: int, target_location_id: int = None, destination_group: int = None):
        goal_msg = MoveToGoal.Goal()
        goal_msg.roscar_id = roscar_id
        # Depending on action, fill the appropriate field
        if target_location_id is not None:
            goal_msg.target_location_id = target_location_id
        if destination_group is not None:
            goal_msg.destination_group = destination_group

        self.get_logger().info(f'Sending MoveToGoal goal to {self._action_name}: ros car={roscar_id}')
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'{self._action_name} goal rejected')
            return
        self.get_logger().info(f'{self._action_name} goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'{self._action_name} result: {result}')
