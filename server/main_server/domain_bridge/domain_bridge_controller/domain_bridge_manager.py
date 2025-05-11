import os
import yaml
import rclpy
import subprocess
from rclpy.node import Node
from shared_interfaces.msg import RoscarRegister

class DomainBridgeManager(Node):
    def __init__(self):
        super().__init__('domain_bridge_manager')
        
        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('config_dir', 'robot_config/domain_bridge')
        self.config_dir = self.get_parameter('config_dir').value
        os.makedirs(self.config_dir, exist_ok=True)

        self.launched = set()
        self.processes = []

        self.declare_parameter('from_domain_id', 25)
        self.declare_parameter('to_domain_id', 26)
        self.from_domain = self.get_parameter('from_domain_id').value
        self.to_domain = self.get_parameter('to_domain_id').value

        # 메시지 구독 설정
        self.subscription = self.create_subscription(
            RoscarRegister,
            'roscar/register',  # 수신할 토픽 이름
            self.listener_callback,
            10
        )

        self.get_logger().info("✅ DomainBridgeManager 초기화 완료")

    def listener_callback(self, msg):
        # 메시지 수신 시 동작
        namespace = msg.roscar_name  # 예: "pinky_0830"
        from_domain = self.from_domain
        to_domain = self.to_domain
        unique_id = f"{namespace}_{from_domain}_to_{to_domain}"

        if unique_id in self.launched:
            self.get_logger().info(f"🔁 이미 실행됨: {unique_id}")
            return

        yaml_filename = f"{namespace}_{from_domain}_to_{to_domain}.yaml"
        yaml_path = os.path.join(self.config_dir, yaml_filename)

        if os.path.exists(yaml_path):
            self.launch_domain_bridge(yaml_path)
            self.launch_sensors(namespace)
            self.launched.add(unique_id)
            self.get_logger().info(f"🟢 Domain bridge + 센서 실행 완료: {namespace}")
        else:
            self.get_logger().error(f"[오류] YAML 파일 없음: {yaml_path}")

    def launch_domain_bridge(self, yaml_path):
        try:
            # 일시적으로 디버깅용 출력 허용
            process = subprocess.Popen(
                ['ros2', 'run', 'domain_bridge', 'domain_bridge', yaml_path],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )

            self.processes.append(process)
            self.get_logger().info(f"🌉 domain_bridge 실행됨 (PID: {process.pid})")
        except Exception as e:
            self.get_logger().error(f"[오류] domain_bridge 실행 실패: {e}")

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
            self.get_logger().error(f"[오류] {reader_name} 실행 실패: {e}")

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
