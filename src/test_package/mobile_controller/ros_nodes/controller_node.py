import rclpy
from rclpy.node import Node
from shared_interfaces.srv import ManualOverride

class ManualOverrideClient(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.cli = self.create_client(ManualOverride, 'manual_override')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service "manual_override"...')
        self.get_logger().info('✅ Connected to manual_override service')

    def send_override_request(self, robot_id, command_text):
        req = ManualOverride.Request()
        req.robot_id = robot_id
        req.command = command_text
        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    client = ManualOverrideClient()

    # 여기에 로봇 ID 및 명령어를 원하는 대로 설정하세요
    robot_id = 1
    command = "STOP"

    client.send_override_request(robot_id, command)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                client.get_logger().info(f"🟢 Service response: success = {response.success}")
            except Exception as e:
                client.get_logger().error(f"❌ Service call failed: {e}")
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()