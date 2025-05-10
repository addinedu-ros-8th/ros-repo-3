import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DomainBridgeManager(Node):
    def __init__(self):
        super().__init__('domain_bridge_manager')
        self.declare_parameter('config_dir', 'robot_config/domain_bridge')
        self.config_dir = self.get_parameter('config_dir').value
        os.makedirs(self.config_dir, exist_ok=True)
        self.launched = set()

        # /robot/register 구독
        self.create_subscription(String, '/robot/register', self.handle_register, 10)

    def handle_register(self, msg):
        """
        로봇에서 register 토픽 수신 시 도메인 브리지 생성
        """
        namespace = msg.data.strip()
        if namespace not in self.launched:
            yaml_path = self.generate_domain_bridge_yaml(namespace)
            self.launch_domain_bridge(yaml_path)
            self.launched.add(namespace)
            self.get_logger().info(f"도메인 브리지 생성 및 실행: {namespace}")

    def generate_domain_bridge_yaml(self, namespace):
        """
        로봇의 네임스페이스 기반 YAML 설정 생성
        """
        topics = [
            f"/roscar/{namespace}/register",
            f"/roscar/{namespace}/sensor/imu",
            f"/roscar/{namespace}/status/battery",
            f"/roscar/{namespace}/sensor/lidar",
            f"/roscar/{namespace}/sensor/ultra"
        ]

        bridge_config = {
            "topics": [{"topic": topic, "type": "std_msgs/msg/String"} for topic in topics]
        }

        yaml_path = os.path.join(self.config_dir, f"{namespace}.yaml")
        with open(yaml_path, 'w') as f:
            yaml.dump(bridge_config, f)

        return yaml_path

    def launch_domain_bridge(self, yaml_path):
        """
        실제 도메인 브리지를 subprocess로 실행
        """
        try:
            process = subprocess.Popen([
                'ros2', 'run', 'domain_bridge', 'domain_bridge_main',
                '--config', yaml_path
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            self.get_logger().info(f"[도메인 브리지 실행됨] PID: {process.pid}, Config: {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"[실패] 도메인 브리지 실행 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    manager = DomainBridgeManager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
