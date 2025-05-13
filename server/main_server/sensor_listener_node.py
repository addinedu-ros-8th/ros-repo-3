import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from shared_interfaces.msg import BatteryStatus, RoscarRegister, SensorData
from datetime import datetime
import json

from db.connect_db import get_roscars_session
from db.roscars_models import RosCars        
from sqlalchemy.dialects.mysql import insert as mysql_insert 

from db.insert_sensor_data import save_sensor_data_to_db  # 모듈화된 DB 저장 함수
from db.parse_sensor_payload import parse_sensor_data     # 모듈화된 JSON 파싱 함수

class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        self.get_logger().info("SensorListener 노드 초기화됨")

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
            f'[BATTERY] name={msg.robot_name}, '
            f'battery={msg.battery_percent:.1f}%, '
            f'is_charging={"YES" if msg.is_charging else "NO"}, '
            f'time={dt.strftime("%Y-%m-%d %H:%M:%S")}'
        )

    def sensor_data_callback(self, msg: SensorData):
        dt = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec * 1e-9)
        self.get_logger().info(f'[SensorData] 수신 - robot_id={msg.robot_id}, time={dt.strftime("%Y-%m-%d %H:%M:%S")}')

        try:
            parsed = parse_sensor_data(msg)
            self.get_logger().info(f'  ▸ LiDAR ranges: {len(parsed["lidar"].get("ranges", []))}개')
            self.get_logger().info(f'  ▸ IMU accel={parsed["imu"].get("accel")}, gyro={parsed["imu"].get("gyro")}, mag={parsed["imu"].get("mag")}')
            self.get_logger().info(f'  ▸ 초음파 거리: {parsed["ultra"].get("front", -1):.2f} cm')
        except Exception as e:
            self.get_logger().warn(f'SensorData JSON 파싱 실패: {e}')
            return

        save_sensor_data_to_db(msg.robot_id, dt, parsed, self.get_logger())

    def register_callback(self, msg):
        self.get_logger().info(
            f'[REGISTER] name={msg.roscar_name}, '
            f'battery={msg.battery_percentage}%, '
            f'status={msg.operational_status}, '
            f'ip={msg.roscar_ip_v4}, '
            f'from_domain_id={msg.from_domain_id}, '
            f'to_domain_id={msg.to_domain_id}'
        )

        # ✅ RosCars 테이블 UPSERT
        session = get_roscars_session()
        try:
            stmt = mysql_insert(RosCars).values(
                roscar_name=msg.roscar_name,
                battery_percentage=msg.battery_percentage,
                roscar_ip_v4=msg.roscar_ip_v4
            )
            stmt = stmt.on_duplicate_key_update(
                battery_percentage=stmt.inserted.battery_percentage,
                roscar_ip_v4=stmt.inserted.roscar_ip_v4
            )
            session.execute(stmt)
            session.commit()
            self.get_logger().info(f"✅ RosCars DB에 '{msg.roscar_name}' 정보 업데이트 완료")
        except Exception as e:
            session.rollback()
            self.get_logger().error(f"❌ RosCars UPSERT 실패: {e}")
        finally:
            session.close()


def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
