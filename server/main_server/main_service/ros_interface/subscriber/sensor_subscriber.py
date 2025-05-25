from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from shared_interfaces.msg import BatteryStatus, RoscarRegister, SensorData
from datetime import datetime
from server.main_server.databases.sensor_utils import parse_sensor_data

class SensorSubscriber(Node):
    def __init__(self, main_ctl_service):
        super().__init__('sensor_subscriber')
        self.main_ctl_service = main_ctl_service

        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.register_sub = self.create_subscription(
            RoscarRegister,
            '/roscar/register',
            self.register_callback,
            10
        )

    def register_callback(self, msg: RoscarRegister):
        ns = msg.roscar_namespace.strip("/")
        self.get_logger().info(
            f'[REGISTER] 수신: {ns}, battery={msg.battery_percentage}%, '
            f'status={msg.operational_status}, ip={msg.roscar_ip_v4}'
        )

        try:
            self.main_ctl_service.schema.update_roscar_status(
                namespace=ns,
                battery=msg.battery_percentage,
                ip=msg.roscar_ip_v4
            )
        except Exception as e:
            self.get_logger().error(f"DB 저장 실패: {e}")

        if ns in self.subscribers:
            return

        battery_topic = f"{ns}/roscar/status/battery"
        sensor_topic  = f"{ns}/roscar/sensor_data"

        battery_sub = self.create_subscription(
            BatteryStatus,
            battery_topic,
            lambda m, n=ns: self.battery_callback(m, n),
            self.qos
        )
        sensor_sub = self.create_subscription(
            SensorData,
            sensor_topic,
            lambda m, n=ns: self.sensor_data_callback(m, n),
            self.qos
        )

        self.subscribers[ns] = (battery_sub, sensor_sub)
        self.get_logger().info(f"[✅ 구독 시작] {battery_topic}, {sensor_topic}")

    def battery_callback(self, msg: BatteryStatus, ns: str):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        dt = datetime.fromtimestamp(t)
        self.get_logger().info(
            f'[BATTERY][{ns}] {msg.battery_percent:.1f}% | '
            f'충전 중: {"YES" if msg.is_charging else "NO"} | '
            f'{dt.strftime("%H:%M:%S")}'
        )
        
    def sensor_data_callback(self, msg: SensorData, ns: str):
        dt = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec * 1e-9)
        self.get_logger().info(
            f'[SensorData][{ns}] roscar_id={msg.roscar_id}, time={dt.strftime("%H:%M:%S")}'
        )

        try:
            parsed = parse_sensor_data(msg)
            self.get_logger().info(f'  ▸ LiDAR {len(parsed["lidar"].get("ranges", []))}개')
            self.get_logger().info(f'  ▸ IMU accel={parsed["imu"].get("accel")}, '
                                f'gyro={parsed["imu"].get("gyro")}, mag={parsed["imu"].get("mag")}')
            self.get_logger().info(f'  ▸ 초음파: {parsed["ultra"].get("front", -1):.2f} cm')
        except Exception as e:
            self.get_logger().warning(f'SensorData 파싱 오류: {e}')
            return

        self.main_ctl_service.schema.save_sensor_data_log(ns, dt, parsed)

