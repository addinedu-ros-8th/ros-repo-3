import sys
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.main_service.main_service.service.log_query_service import LogQueryService
from server.main_server.main_service.main_service.publisher.log_event_publisher import LogEventPublisher
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.utils import SensorUtils
from server.main_server.main_service.comm.controller import RuntimeController
from server.main_server.main_service.comm.service import MainService
from server.main_server.main_service.handler.login_handler import handle_login_request
from server.main_server.main_service.handler.qrcode_handler import handle_qrcode_search
from server.main_server.main_service.handler.delivery_handler import (
    handle_delivery_request,
    handle_cancel_task
)
from server.main_server.main_service.handler.task_handler import (
    handle_task_result_check,
    send_task_status_notification
)
from server.main_server.main_service.handler.ai_handler import handle_ai_result
from server.main_server.main_service.comm.tcp_handler import TCPHandler
from server.main_server.main_service.comm.tcp_message_router import MessageRouter
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.schema_manager import SchemaManager

def create_executor(db: DatabaseManager):
    logger = RoscarsLogWriter(db.get_session("roscars_log"))
    sensor_node = SensorUtils(logger, db)
    log_query_node = LogQueryService()
    log_event_publisher = LogEventPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_node)
    executor.add_node(log_query_node)
    executor.add_node(log_event_publisher)
    return executor

def main():
    # 1) 런타임 컨트롤러 및 DB 초기화
    runtime = RuntimeController()
    db = DatabaseManager()
    SchemaManager(db).check_db_init()
    print("[MAIN] DB 연결 및 구조 확인 완료")

    # 2) 서비스 준비
    service = MainService(db)
    service.register_shutdown_flag(runtime.shutdown_flag)

    # 3) 라우터에 핸들러 등록
    router = MessageRouter(service)
    router.register("AU", handle_login_request)
    router.register("IS", handle_qrcode_search)
    router.register("IR", handle_delivery_request)
    router.register("CD", handle_cancel_task)
    router.register("TR", handle_task_result_check)
    router.register_notify("TR_NOTIFY", send_task_status_notification)
    router.register("IN", handle_ai_result)

    # 4) TCP 서버 실행
    ai_test = "--ai-test" in sys.argv
    if ai_test:
        service.enable_shutdown_after_ai_result = True

    port = 5001 if ai_test else 9000
    tcp_server = TCPHandler("0.0.0.0", port, router)
    tcp_thread = threading.Thread(target=tcp_server.start_server, daemon=True)
    tcp_thread.start()
    print(f"[MAIN] TCP 서버 시작 (port={port})")

    # 5) ROS2 노드 스핀
    rclpy.init()
    executor = create_executor(db)

    if "--test" in sys.argv:
        runtime.enable_auto_shutdown(3)

    try:
        while not runtime.shutdown_flag.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        print("[MAIN] 종료 처리 중...")
        executor.shutdown()
        rclpy.shutdown()
        tcp_thread.join(timeout=2)
        print("[MAIN] 종료 완료")


if __name__ == "__main__":
    main()
    