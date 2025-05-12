import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from shared_interfaces.msg import (
    ImuStatus,
    LidarScan,
    RoscarRegister,
    BatteryStatus  # ✅ 최신 명세 기준 메시지
)

class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        self.get_logger().info("✅ SensorListener 노드 초기화됨")

        # 초음파
        self.ultra_sub = self.create_subscription(
            Float32,
            '/pinky_07db/roscar/sensor/ultra',
            self.ultra_callback,
            10
        )

        # 배터리
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/pinky_07db/roscar/status/battery',  # ✅ 토픽 경로 변경됨
            self.battery_callback,
            10
        )

        # IMU
        self.imu_sub = self.create_subscription(
            ImuStatus,
            '/pinky_07db/roscar/sensor/imu',
            self.imu_callback,
            10
        )

        # LiDAR
        self.lidar_sub = self.create_subscription(
            LidarScan,
            '/pinky_07db/roscar/sensor/lidar',
            self.lidar_callback,
            10
        )

        # Register
        self.register_sub = self.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.register_callback,
            10
        )

    def ultra_callback(self, msg):
        self.get_logger().info(f'[ULTRA] 거리: {msg.data:.2f} cm')

    def battery_callback(self, msg):
        self.get_logger().info(
            f'[BATTERY] robot_id={msg.robot_id}, '
            f'battery={msg.battery_percent:.1f}%, '
            f'is_charging={"YES" if msg.is_charging else "NO"}, '
            f'time={msg.stamp.sec}.{msg.stamp.nanosec}'
        )

    def imu_callback(self, msg):
        self.get_logger().info(
            f'[IMU] accel=({msg.accel_x:.2f}, {msg.accel_y:.2f}, {msg.accel_z:.2f}), '
            f'gyro=({msg.gyro_x:.2f}, {msg.gyro_y:.2f}, {msg.gyro_z:.2f}), '
            f'mag=({msg.mag_x:.2f}, {msg.mag_y:.2f}, {msg.mag_z:.2f})'
        )

    def lidar_callback(self, msg):
        self.get_logger().info(f'[LIDAR] angle_min={msg.angle_min}, ranges_len={len(msg.ranges)}')

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
