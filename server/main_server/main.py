import os
import sys
import rclpy

# 현재 디렉토리를 Python 경로에 추가
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from db import connect_db
from sensor_reader import battery_reader, imu_reader, lidar_reader, ultra_reader, roscar_register_listner
from domain_bridge import domain_bridge_manager, launch_domain_bridge

def main():
    print("모든 모듈을 성공적으로 import 했습니다.")

    connect_db.check_db_init()
    rclpy.init()

    # 노드 객체 생성
    domain_node = domain_bridge_manager.DomainBridgeManager()
   # roscar_node = roscar_register_listner.RobotRegisterRequester()

    # 여러 노드를 동시에 돌리려면 executor 사용
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(domain_node)
    #executor.add_node(roscar_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        domain_node.destroy_node()
       # roscar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
