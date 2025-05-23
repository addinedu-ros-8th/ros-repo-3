import rclpy
from rclpy.node import Node

from shared_interfaces.msg import (
    NavigationGoal,
    ChargeCommand,
    EmergencyCommand,
    ObstacleAvoidanceCmd,
    PrecisionStopCmd,
)

class ControlCommandPublisher(Node):
    def __init__(self):
        super().__init__('control_command_publisher')
        self.get_logger().info("[ControlPublisher] 노드 초기화 완료")

        self.nav_pub = self.create_publisher(NavigationGoal, '/roscar/goal/navigation', 10)
        self.charge_pub = self.create_publisher(ChargeCommand, '/roscar/command/charge', 10)
        self.emergency_pub = self.create_publisher(EmergencyCommand, '/roscar/command/emergency', 10)
        self.obstacle_pub = self.create_publisher(ObstacleAvoidanceCmd, '/roscar/command/obstacle', 10)
        self.precision_pub = self.create_publisher(PrecisionStopCmd, '/roscar/command/precision_stop', 10)

    def send_navigation_goal(self, roscar_id: int, x: float, y: float, theta: float):
        msg = NavigationGoal()
        msg.roscar_id = roscar_id
        msg.goal_x = x
        msg.goal_y = y
        msg.theta = theta
        self.nav_pub.publish(msg)
        self.get_logger().info(f"[NAV] Goal → ID={roscar_id}, x={x}, y={y}, θ={theta}")

    def send_charge_command(self, roscar_id: int, x: float, y: float, theta: float):
        msg = ChargeCommand()
        msg.roscar_id = roscar_id
        msg.goal_x = x
        msg.goal_y = y
        msg.theta = theta
        self.charge_pub.publish(msg)
        self.get_logger().info(f"[CHARGE] → ID={roscar_id}, x={x}, y={y}, θ={theta}")

    def send_emergency_command(self, roscar_id: int, mode: int, reason: str):
        msg = EmergencyCommand()
        msg.roscar_id = roscar_id
        msg.emergency_mode = mode  # 0: 정상, 1: 정지, 2: 재시작
        msg.reason = reason
        self.emergency_pub.publish(msg)
        self.get_logger().info(f"[EMERGENCY] → ID={roscar_id}, mode={mode}, reason={reason}")

    def send_obstacle_command(self, roscar_id: int, direction: float, speed: float):
        msg = ObstacleAvoidanceCmd()
        msg.roscar_id = roscar_id
        msg.direction = direction
        msg.speed = speed
        self.obstacle_pub.publish(msg)
        self.get_logger().info(f"[OBSTACLE] → ID={roscar_id}, dir={direction}, speed={speed}")

    def send_precision_stop(self, roscar_id: int, x: float, y: float):
        msg = PrecisionStopCmd()
        msg.roscar_id = roscar_id
        msg.target_x = x
        msg.target_y = y
        self.precision_pub.publish(msg)
        self.get_logger().info(f"[PRECISION_STOP] → ID={roscar_id}, x={x}, y={y}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlCommandPublisher()

    try:
        # 예시: 로봇 1번에게 목적지 (2.0, 3.0, 1.57 rad) 명령 전송
        node.send_navigation_goal(roscar_id=1, x=2.0, y=3.0, theta=1.57)

        # ROS2가 토픽 송신을 완료할 수 있도록 약간의 시간 확보
        rclpy.spin_once(node, timeout_sec=1.0)

    finally:
        node.destroy_node()
        rclpy.shutdown()

