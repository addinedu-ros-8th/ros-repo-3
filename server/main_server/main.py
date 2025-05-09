import sys
import os

# 프로젝트 루트 디렉토리 경로 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from ros_nodes.service_node import MainService
from logger import log_info, log_error
from db.init_db import init_db
# from db.connect_db import get_roscars_session, get_roscars_log_session

def main():
    try:
        # 데이터베이스 초기화
        init_db()

        ### DB 연결
        # roscars_session = get_roscars_session()
        # roscars_log_session = get_roscars_log_session()

        service = MainService()
        service.start()
    except KeyboardInterrupt:
        log_info("[Main] Main Service interrupted by user.")
    except Exception as e:
        log_error(f"[Main] Exception occurred: {str(e)}")

if __name__ == "__main__":
    main()
