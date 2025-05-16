import signal
import threading
import rclpy

from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.schema_manager import SchemaManager  
from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.databases.utils import SensorUtils
from server.main_server.main_service.core.main_service import MainService
from server.main_server.main_service.tcp_handler import TCPHandler
from server.main_server.main_service.core.shutdown import shutdown_flag
from server.main_server.main_service.core.message_router import MessageRouter

router = MessageRouter()
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

def main(main_test_mode=False, ai_test_mode=False):
    db = DatabaseManager()
    schema = SchemaManager(db)
    schema.check_db_init()
    print("[MAIN] DB 연결 및 구조 확인 완료")

    main_service = MainService()

    if ai_test_mode:
        main_service.enable_shutdown_after_ai_result = True

    port = 5001 if ai_test_mode else 9000
    tcp_server = TCPHandler("0.0.0.0", port, router)
    tcp_thread = threading.Thread(target=tcp_server.start_server, daemon=True)
    tcp_thread.start()
    print(f"[MAIN] TCP 서버 시작 (port={port})")

    rclpy.init()
    logger = RoscarsLogWriter(db.get_session("roscars_log"))
    ros_node = SensorUtils(logger, db)

    if main_test_mode:
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
    main(
        main_test_mode="--test" in sys.argv,
        ai_test_mode="--ai-test" in sys.argv
    )
