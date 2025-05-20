import rclpy
from rclpy.executors import MultiThreadedExecutor
from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.main_service.ros_interface.service.log_query_service import LogQueryService
from server.main_server.main_service.ros_interface.publisher.log_event_publisher import LogEventPublisher
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.utils import SensorUtils


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