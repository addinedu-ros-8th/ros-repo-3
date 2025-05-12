import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from shared_interfaces.msg import (
    BatteryStatus,
    RoscarRegister,
    SensorData  # ✅ 통합 메시지
)
from datetime import datetime
import json

class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        self.get_logger().info("✅ SensorListener 노드 초기화됨")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # 배터리 메시지
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/pinky_07db/roscar/status/battery',
            self.battery_callback,
            qos
        )

        # 통합 SensorData 토픽
        self.sensor_data_sub = self.create_subscription(
            SensorData,
            '/pinky_07db/roscar/sensor_data',
            self.sensor_data_callback,
            qos
        )

        # Register 메시지
        self.register_sub = self.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.register_callback,
            10
        )

    def battery_callback(self, msg):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        dt = datetime.fromtimestamp(t)

        self.get_logger().info(
            f'[BATTERY] name={msg.robot_name}, '
            f'battery={msg.battery_percent:.1f}%, '
            f'is_charging={"YES" if msg.is_charging else "NO"}, '
            f'time={dt.strftime("%Y-%m-%d %H:%M:%S")}'
        )

    def sensor_data_callback(self, msg: SensorData):
        # timestamp 출력
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        dt = datetime.fromtimestamp(t).strftime('%Y-%m-%d %H:%M:%S')
        self.get_logger().info(f'[SensorData] 수신 - robot_id={msg.robot_id}, time={dt}')


        # ✅ LiDAR
        try:
            lidar = json.loads(msg.lidar_raw)
            ranges = lidar.get("ranges", [])
            self.get_logger().info(f'  ▸ LiDAR ranges: {len(ranges)}개')
        except Exception as e:
            self.get_logger().warn(f'  ❌ LiDAR JSON 파싱 실패: {e}')

        # ✅ IMU
        try:
            imu = json.loads(msg.imu_data)
            a = imu.get("accel", [0, 0, 0])
            g = imu.get("gyro", [0, 0, 0])
            m = imu.get("mag", [0, 0, 0])
            self.get_logger().info(
                f'  ▸ IMU accel={a}, gyro={g}, mag={m}'
            )
        except Exception as e:
            self.get_logger().warn(f'  ❌ IMU JSON 파싱 실패: {e}')

        # ✅ 초음파
        try:
            ultra = json.loads(msg.ultrasonic_data)
            self.get_logger().info(f'  ▸ 초음파 거리: {ultra.get("front", -1):.2f} cm')
        except Exception as e:
            self.get_logger().warn(f'  ❌ 초음파 JSON 파싱 실패: {e}')

    def register_callback(self, msg):
        self.get_logger().info(
            f'[REGISTER] name={msg.roscar_name}, '
            f'battery={msg.battery_percentage}%, '
            f'status={msg.operational_status}, '
            f'ip={msg.roscar_ip_v4}, '
            f'from_domain_id={msg.from_domain_id}, '
            f'to_domain_id={msg.to_domain_id}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
