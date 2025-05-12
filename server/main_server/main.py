import os
import sys
import rclpy

# -------------------------------
# 경로 설정
# -------------------------------

# 현재 파일 위치: ~/ros-repo-3/server/main_server/main.py
# 목표: server/main_server 및 server/ 둘 다 sys.path에 포함

current_dir = os.path.abspath(os.path.dirname(__file__))  # .../main_server
project_root = os.path.abspath(os.path.join(current_dir, '..'))  # .../server

for path in [current_dir, project_root]:
    if path not in sys.path:
        sys.path.insert(0, path)  # 맨 앞에 추가하여 ROS 패키지보다 우선

# -------------------------------
# 모듈 import
# -------------------------------

from db import connect_db
from sensor_reader import battery_reader, imu_reader, lidar_reader, ultra_reader, roscar_register_listner
# from domain_bridge.domain_bridge_controller import domain_bridge_manager, launch_domain_bridge

# -------------------------------
# 메인 실행 함수
# -------------------------------

def main():
    print("모든 모듈을 성공적으로 import 했습니다.")

    # DB 초기화
    connect_db.check_db_init()

    # ROS2 초기화
    rclpy.init()

    # 노드 생성
    # domain_node = domain_bridge_manager.DomainBridgeManager()
    # roscar_node = roscar_register_listner.RobotRegisterRequester()

    # 다중 실행을 위한 Executor 구성
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # executor.add_node(domain_node)
    # executor.add_node(roscar_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # domain_node.destroy_node()
        # roscar_node.destroy_node()
        rclpy.shutdown()


# -------------------------------
# 실행 진입점
# -------------------------------

if __name__ == '__main__':
    main()
