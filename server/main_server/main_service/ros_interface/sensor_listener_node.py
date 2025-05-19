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
        self.get_logger().info("SensorListener 노드 초기화됨")

        self.db_manager = db_manager
        self.db_logger = db_logger
        self.sensor_utils = SensorUtils(db_logger=db_logger, db_manager=db_manager)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/pinky_07db/roscar/status/battery',
            self.battery_callback,
            qos
        )

        self.sensor_data_sub = self.create_subscription(
            SensorData,
            '/pinky_07db/roscar/sensor_data',
            self.sensor_data_callback,
            qos
        )

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
            f'[BATTERY] name={msg.roscar_namespace}, '
            f'battery={msg.battery_percent:.1f}%, '
            f'is_charging={"YES" if msg.is_charging else "NO"}, '
            f'time={dt.strftime("%Y-%m-%d %H:%M:%S")}'
        )

    def sensor_data_callback(self, msg: SensorData):
        dt = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec * 1e-9)
        self.get_logger().info(f'[SensorData] 수신 - roscar_id={msg.roscar_id}, time={dt.strftime("%Y-%m-%d %H:%M:%S")}')

        try:
            parsed = self.sensor_utils.parse_sensor_data(msg)
            self.get_logger().info(f'  ▸ LiDAR ranges: {len(parsed["lidar"].get("ranges", []))}개')
            self.get_logger().info(f'  ▸ IMU accel={parsed["imu"].get("accel")}, gyro={parsed["imu"].get("gyro")}, mag={parsed["imu"].get("mag")}')
            self.get_logger().info(f'  ▸ 초음파 거리: {parsed["ultra"].get("front", -1):.2f} cm')
        except Exception as e:
            self.get_logger().warn(f'SensorData JSON 파싱 실패: {e}')
            return

        self.sensor_utils.save_sensor_data_to_db(msg.roscar_id, dt, parsed)

    def register_callback(self, msg):
        self.get_logger().info(
            f'[REGISTER] name={msg.roscar_namespace}, '
            f'battery={msg.battery_percentage}%, '
            f'status={msg.operational_status}, '
            f'ip={msg.roscar_ip_v4}, '
            f'from_domain_id={msg.from_domain_id}, '
            f'to_domain_id={msg.to_domain_id}'
        )

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
                self.get_logger().info(f"RosCars DB에 '{msg.roscar_namespace}' 정보 업데이트 완료")
        except Exception as e:
            self.get_logger().error(f"RosCars UPSERT 실패: {e}")


def main(args=None):
    rclpy.init(args=args)

    db_manager = DatabaseManager()
    db_logger = RoscarsLogWriter(db_manager)

    node = SensorListener(db_manager, db_logger)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
