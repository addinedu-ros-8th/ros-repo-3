import signal
import threading
import rclpy
from rclpy.executors import ExternalShutdownException

from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.schema_manager import SchemaManager  
from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.databases.utils import SensorUtils
from server.main_server.main_service.main_service import MainService 
from server.main_server.network.tcp_handler import TCPHandler


shutdown_flag = threading.Event()

def signal_handler(sig, frame):
    print("[MAIN] 종료 요청 수신 (SIGINT)")
    shutdown_flag.set()

signal.signal(signal.SIGINT, signal_handler)


def auto_shutdown_after(seconds):
    def _shutdown():
        print(f"[TEST] {seconds}초 경과 → 종료 신호 발생")
        shutdown_flag.set()
    timer = threading.Timer(seconds, _shutdown)
    timer.start()


def main(test_mode=False):
    db = DatabaseManager()
    schema = SchemaManager(db)
    schema.check_db_init()
    print("[MAIN] DB 연결 및 구조 확인 완료")

    main_service = MainService()
    tcp_server = TCPHandler("0.0.0.0", 9000, main_service)
    tcp_thread = threading.Thread(target=tcp_server.start_server, daemon=True)
    tcp_thread.start()
    print("[MAIN] TCP 서버 시작")

    rclpy.init()
    logger = RoscarsLogWriter(db.get_session("roscars_log"))
    ros_node = SensorUtils(logger, db)

    if test_mode:
        auto_shutdown_after(3)

    try:
        while not shutdown_flag.is_set() and rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)

    except rclpy.executors.ExternalShutdownException:
        print("[MAIN] ROS2가 종료되어 spin 종료됨")

    finally:
        print("[MAIN] 종료 처리 중...")
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        tcp_server.shutdown_server()
        tcp_thread.join(timeout=2)
        print("[MAIN] 종료 완료")


if __name__ == '__main__':
    import sys
    main(test_mode="--test" in sys.argv)
