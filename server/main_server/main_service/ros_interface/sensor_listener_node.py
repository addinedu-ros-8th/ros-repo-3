import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from shared_interfaces.msg import BatteryStatus, RoscarRegister, SensorData
from datetime import datetime
from sqlalchemy.dialects.mysql import insert as mysql_insert

from server.main_server.databases.utils import SensorUtils
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.databases.models.roscars_models import RosCars

class SensorListener(Node):
    def __init__(self, db_manager: DatabaseManager, db_logger: RoscarsLogWriter):
        super().__init__('sensor_listener')
        self.get_logger().info("SensorListener 노드 시작됨 (글로벌 토픽 대기 중)")

        self.db_manager = db_manager
        self.db_logger = db_logger
        self.sensor_utils = SensorUtils(db_logger=db_logger, db_manager=db_manager)
        self.subscribers = {}  # namespace -> (battery_sub, sensor_sub)

        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # 🌐 글로벌 register 토픽 구독
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

        # ✅ DB upsert
        try:
            with self.db_manager.session_scope("roscars") as session:
                stmt = mysql_insert(RosCars).values(
                    roscar_namespace=msg.roscar_namespace,
                    battery_percentage=msg.battery_percentage,
                    roscar_ip_v4=msg.roscar_ip_v4
                )
                stmt = stmt.on_duplicate_key_update(
                    battery_percentage=stmt.inserted.battery_percentage,
                    roscar_ip_v4=stmt.inserted.roscar_ip_v4
                )
                session.execute(stmt)
                self.get_logger().info(f"DB 업데이트 완료: '{msg.roscar_namespace}'")
        except Exception as e:
            self.get_logger().error(f"DB 저장 실패: {e}")

        # ✅ 이미 구독했으면 무시
        if ns in self.subscribers:
            return

        # ✅ 동적 구독 생성
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
            parsed = self.sensor_utils.parse_sensor_data(msg)
            self.get_logger().info(f'  ▸ LiDAR {len(parsed["lidar"].get("ranges", []))}개')
            self.get_logger().info(f'  ▸ IMU accel={parsed["imu"].get("accel")}, '
                                   f'gyro={parsed["imu"].get("gyro")}, mag={parsed["imu"].get("mag")}')
            self.get_logger().info(f'  ▸ 초음파: {parsed["ultra"].get("front", -1):.2f} cm')
        except Exception as e:
            self.get_logger().warn(f'SensorData 파싱 오류: {e}')
            return

        self.sensor_utils.save_sensor_data_to_db(msg.roscar_id, dt, parsed)


def main(args=None):
    rclpy.init(args=args)
    db_manager = DatabaseManager()
    db_logger = RoscarsLogWriter(db_manager)

    node = SensorListener(db_manager, db_logger)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
