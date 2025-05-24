import sys
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

from server.main_server.main_service.ros_interface.action.maintenance_charge_action_client import MaintenanceChargeActionClient
from server.main_server.main_service.ros_interface.action.move_to_goal_client import MoveToGoalClient
from server.main_server.main_service.ros_interface.action.scan_inventory_action_client import ScanInventoryActionClient
from server.main_server.main_service.ros_interface.action.security_patrol_action_client import SecurityPatrolActionClient
from server.main_server.main_service.ros_interface.action.start_delivery_client import StartDeliveryClient
from server.main_server.main_service.ros_interface.service.log_query_service import LogQueryService
from server.main_server.main_service.ros_interface.service.manager_login_service import ManagerLoginService
from server.main_server.main_service.ros_interface.service.roscar_status_service import QueryRoscarStatusService
from server.main_server.main_service.ros_interface.publisher.avoidance_cmd_publisher import AvoidanceCmdPublisher
from server.main_server.main_service.ros_interface.publisher.charge_command_publisher import ChargeCommandPublisher
from server.main_server.main_service.ros_interface.publisher.control_command_publisher import ControlCommandPublisher
from server.main_server.main_service.ros_interface.publisher.dashboard_status_publisher import DashboardStatusPublisher
from server.main_server.main_service.ros_interface.publisher.log_event_publisher import LogEventPublisher
from server.main_server.main_service.ros_interface.publisher.map_pose_publisher import MapPosePublisher
from server.main_server.main_service.ros_interface.publisher.navigation_goal_publisher import NavigationGoalPublisher
from server.main_server.main_service.ros_interface.publisher.obstacle_response_publisher import ObstacleResponsePublisher
from server.main_server.main_service.ros_interface.publisher.precision_stop_cmd_publisher import PrecisionStopCmdPublisher
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.main_service.comm.controller import RuntimeController
from server.main_server.main_service.comm.ctl_service import MainControlService
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

from server.main_server.main_service.ros_interface.subscriber.obstacle_detected_subscriber import ObstacleDetectedSubscriber
from server.main_server.main_service.ros_interface.subscriber.precision_stop_result_subscriber import PrecisionStopResultSubscriber
from server.main_server.main_service.ros_interface.subscriber.roscar_status_subscriber import RoscarStatusSubscriber
from server.main_server.main_service.ros_interface.subscriber.sensor_subscriber import SensorSubscriber
from server.main_server.main_service.ros_interface.subscriber.task_complete_subscriber import TaskCompleteSubscriber
from server.main_server.main_service.ros_interface.subscriber.task_progress_subscriber import TaskProgressSubscriber

def create_executor(main_ctl_service):
    # # Action Clients
    # maintenance_charge_node = MaintenanceChargeActionClient()
    # move_to_goal_node = MoveToGoalClient()
    # scan_inventory_node = ScanInventoryActionClient()
    # security_patrol_node = SecurityPatrolActionClient()
    # start_delivery_node = StartDeliveryClient()

    # Services
    log_query_node = LogQueryService(main_ctl_service)
    manager_login_node = ManagerLoginService(main_ctl_service)
    status_service_node = QueryRoscarStatusService(main_ctl_service) 

    # Publishers
    avoidance_cmd_publisher = AvoidanceCmdPublisher()
    charge_command_publisher = ChargeCommandPublisher()
    control_command_publisher = ControlCommandPublisher()
    dashboard_status_publisher = DashboardStatusPublisher()
    log_event_publisher = LogEventPublisher()
    map_pose_publisher = MapPosePublisher()
    navigation_goal_publisher = NavigationGoalPublisher()
    obstacle_response_publisher = ObstacleResponsePublisher()
    precision_stop_cmd_publisher = PrecisionStopCmdPublisher()

    # Subscribers
    obstacle_subscriber = ObstacleDetectedSubscriber()
    precision_stop_result_subscriber = PrecisionStopResultSubscriber()
    roscar_status_subscriber = RoscarStatusSubscriber(main_ctl_service)
    sensor_subscriber = SensorSubscriber(main_ctl_service)
    task_complete_subscriber = TaskCompleteSubscriber()
    task_progress_subscriber = TaskProgressSubscriber()

    executor = MultiThreadedExecutor()

    # executor.add_node(maintenance_charge_node)
    # executor.add_node(move_to_goal_node)
    # executor.add_node(scan_inventory_node)
    # executor.add_node(security_patrol_node)
    # executor.add_node(start_delivery_node)

    executor.add_node(avoidance_cmd_publisher)
    executor.add_node(charge_command_publisher)
    executor.add_node(control_command_publisher)
    executor.add_node(dashboard_status_publisher)
    executor.add_node(log_query_node)
    executor.add_node(map_pose_publisher)
    executor.add_node(navigation_goal_publisher)
    executor.add_node(obstacle_response_publisher)
    executor.add_node(precision_stop_cmd_publisher)

    executor.add_node(manager_login_node)
    executor.add_node(status_service_node)
    executor.add_node(log_event_publisher)
    executor.add_node(obstacle_subscriber)
    executor.add_node(precision_stop_result_subscriber)
    executor.add_node(roscar_status_subscriber)
    executor.add_node(sensor_subscriber)
    executor.add_node(task_complete_subscriber)
    executor.add_node(task_progress_subscriber)

    return executor


def main():
    # 1) 런타임 컨트롤러 및 DB 초기화
    runtime = RuntimeController()
    db = DatabaseManager()
    SchemaManager(db).check_db_init()
    print("[MAIN] DB 연결 및 구조 확인 완료")

    # 2) 서비스 준비
    main_ctl_service = MainControlService(db)
    main_ctl_service.register_shutdown_flag(runtime.shutdown_flag)

    # 3) 라우터에 핸들러 등록
    router = MessageRouter(main_ctl_service)
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
        main_ctl_service.enable_shutdown_after_ai_result = True

    port = 5001 if ai_test else 9000
    tcp_server = TCPHandler("0.0.0.0", port, router)
    tcp_thread = threading.Thread(target=tcp_server.start_server, daemon=True)
    tcp_thread.start()
    print(f"[MAIN] TCP 서버 시작 (port={port})")

    # 5) ROS2 노드 스핀
    rclpy.init()
    executor = create_executor(main_ctl_service)

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
