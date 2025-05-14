import rclpy
from databases.database_manager import DatabaseManager
from databases.schema_manager import SchemaManager
from network.ros_nodes.sensor_listener_node import SensorListener

def main():
    # DB 연결 및 구조 확인
    try:
        db = DatabaseManager()
        schema = SchemaManager(db)
        schema.check_db_init()
        print("[MAIN] DB 연결 및 구조 확인 완료")
    except Exception as e:
        print(f"[MAIN] DB 연결 실패: {e}")
        return

    # ROS2 노드 실행
    rclpy.init()
    node = SensorListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[MAIN] 종료 요청 수신 (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[MAIN] ROS2 종료")

if __name__ == '__main__':
    main()
