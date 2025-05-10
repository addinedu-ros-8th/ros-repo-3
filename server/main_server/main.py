import os
import sys

# 현재 디렉토리를 Python 경로에 추가
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# 프로젝트의 하위 폴더들을 패키지로 인식시키기 위해 필요한 임포트
from db import connect_db
from ros_nodes.roscar_register_listner import RobotRegisterRequester
from sensor_reader import battery_reader, imu_reader, lidar_reader, ultra_reader
from domain_bridge import domain_bridge_manager, launch_domain_bridge

# 여기서부터 main.py에서 사용하려는 로직 작성
def main():
    # 예시 로직을 추가해주세요
    print("모든 모듈을 성공적으로 import 했습니다.")

    # 예시 함수 호출 (필요에 맞게 구현)
    connect_db.check_db_init()

if __name__ == '__main__':
    main()
