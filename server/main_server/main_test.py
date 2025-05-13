# main_test.py

import os
import sys
import rclpy

current_dir = os.path.abspath(os.path.dirname(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))

for path in [current_dir, project_root]:
    if path not in sys.path:
        sys.path.insert(0, path)

from sensor_listener_node import SensorListener
from db import connect_db

def main():
    print("DB 연결 확인 중...")
    try:
        connect_db.check_db_init()
        print("DB 연결 및 구조 확인 완료")
    except Exception as e:
        print(f"DB 연결 실패: {e}")
        return

    rclpy.init()
    node = SensorListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
