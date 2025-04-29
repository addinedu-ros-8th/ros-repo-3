import sys
import os

# 프로젝트 루트 디렉토리 경로 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from viewer.manager.main import MainWindow  # 절대 경로로 수정
from viewer.theme import apply_theme
from ros_nodes.service_node import MainService
from logger import log_info, log_error

def main():
    try:
        log_info("[Main] Starting Main Service...")
        service = MainService()
        service.start()
    except KeyboardInterrupt:
        log_info("[Main] Main Service interrupted by user.")
    except Exception as e:
        log_error(f"[Main] Exception occurred: {str(e)}")

if __name__ == "__main__":
    main()
