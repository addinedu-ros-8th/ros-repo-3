import rclpy as rp
from rclpy.node import Node
from shared_interfaces.msg import LidarScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class SLLidarReader(Node):
    def __init__(self):
        super().__init__('sllidar_reader')

        self.subscription = self.create_subscription(
            LidarScan,
            '/roscar/sensor/lidar',
            self.callback,
            qos_profile
        )

        self.is_printing = False

    def callback(self, msg):
        total_angle = (msg.angle_max - msg.angle_min)
        expected_count = int(total_angle / msg.angle_increment)

        if len(msg.ranges) >= expected_count:
            # 로스카 이름 출력 (로스카 프레임 헤더)
            self.get_logger().info(f"\n[roscar: {msg.roscar_name}]\n" + "-"*30)
            
            # 거리 정보 출력
            output_lines = []
            for i, range_value in enumerate(msg.ranges):
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad) % 360
                if range_value != float('inf'):
                    output_lines.append(f"Angle {angle_deg:.2f}°: {range_value:.2f} m")

            # 모두 한 번에 출력
            full_output = "\n".join(output_lines)
            self.get_logger().info(full_output + "\n" + "="*30)


def main(args=None):
    rp.init(args=args)
    node = SLLidarReader()
    rp.spin(node)
    rp.shutdown()

if __name__ == '__main__':
    main()
