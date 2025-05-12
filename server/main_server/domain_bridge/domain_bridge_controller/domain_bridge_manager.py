import os
import rclpy
import subprocess
from rclpy.node import Node
from shared_interfaces.msg import RoscarRegister

class DomainBridgeManager(Node):
    def __init__(self):
        super().__init__('domain_bridge_manager')

        # 파라미터 선언
        self.declare_parameter(
            'config_dir',
            'server/main_server/domain_bridge/roscars_config/domain_bridge'
        )
        self.declare_parameter('from_domain_id', 36)
        self.declare_parameter('to_domain_id', 14)

        self.config_dir = self.get_parameter('config_dir').value
        os.makedirs(self.config_dir, exist_ok=True)

        self.from_domain = self.get_parameter('from_domain_id').value
        self.to_domain = self.get_parameter('to_domain_id').value

        self.launched = set()
        self.processes = []

        # /roscar/register 구독 시작
        self.subscription = self.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.listener_callback,
            10
        )

        self.get_logger().info("✅ DomainBridgeManager 초기화 완료")

    def listener_callback(self, msg):
        self.get_logger().info(f"📥 수신됨: {msg.roscar_name} (from {msg.from_domain_id} → to {msg.to_domain_id})")

        namespace = msg.roscar_name
        unique_id = f"{namespace}_{self.from_domain}_to_{self.to_domain}"

        if unique_id in self.launched:
            self.get_logger().info(f"🔁 이미 실행됨: {unique_id}")
            return

        yaml_filename = f"{namespace}_{self.from_domain}_to_{self.to_domain}.yaml"
        yaml_path = os.path.abspath(os.path.join(self.config_dir, yaml_filename))

        if os.path.exists(yaml_path):
            self.launch_domain_bridge(yaml_path)
            self.launch_sensors(namespace)
            self.launched.add(unique_id)
            self.get_logger().info(f"🟢 Bridge + 센서 실행 완료: {unique_id}")
        else:
            self.get_logger().error(f"[❌] YAML 파일 없음: {yaml_path}")

    def launch_domain_bridge(self, yaml_path):
        try:
            process = subprocess.Popen(
                [
                    'ros2', 'launch', 'domain_bridge', 'domain_bridge.launch.xml',
                    f'config:={yaml_path}'
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(process)
            self.get_logger().info(f"🌉 domain_bridge 실행됨 (PID: {process.pid})")
        except Exception as e:
            self.get_logger().error(f"[❌] domain_bridge 실행 실패: {e}")

    def launch_sensors(self, namespace):
        sensors = ['battery_reader', 'lidar_reader', 'imu_reader', 'ultra_reader']
        for sensor in sensors:
            self.launch_reader(namespace, sensor)

    def launch_reader(self, namespace, reader_name):
        try:
            process = subprocess.Popen(
                ['ros2', 'run', 'sensor_reader', reader_name, namespace],
                stdout=None,
                stderr=None
            )
            self.processes.append(process)
            self.get_logger().info(f"📡 센서 실행됨: {reader_name} (PID: {process.pid})")
        except Exception as e:
            self.get_logger().error(f"[❌] {reader_name} 실행 실패: {e}")

    def check_processes(self):
        for process in self.processes[:]:
            if process.poll() is not None:
                self.get_logger().warn(f"⚠️ 종료된 프로세스 감지 (PID: {process.pid})")
                self.processes.remove(process)

def main(args=None):
    rclpy.init(args=args)
    manager = DomainBridgeManager()
    try:
        while rclpy.ok():
            rclpy.spin_once(manager, timeout_sec=0.5)
            manager.check_processes()
    finally:
        for p in manager.processes:
            p.terminate()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
