import rclpy
from rclpy.node import Node
import json

from shared_interfaces.srv import LogQuery
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.query import MainServiceQuery

class LogQueryService(Node):
    def __init__(self):
        super().__init__('log_query_service')
        self.srv = self.create_service(LogQuery, '/log/request/query', self.handle_query)

        db = DatabaseManager()
        self.query = MainServiceQuery(
            db.get_session("roscars"),
            db.get_session("roscars_log")
        )

        self.query_map = {
            "delivery": self.query.get_all_delivery_logs,
            "task": self.query.get_task_event_history,
            "roscar": self.query.get_roscar_event_detail,
            "precision_stop": self.query.get_precision_stop_result,
            "trajectory": self.query.get_roscar_trajectory,
            "driving_event": self.query.get_roscar_driving_event_log,
            "sensor_fusion": self.query.get_sensor_for_training,
            "control_command": self.query.get_control_command_log,
            "filesystem": self.query.get_filesystem_log,
            "rack_sensor": self.query.get_rack_sensor_log
        }

    def handle_query(self, request, response):
        query_type = request.query_type.strip().lower()
        self.get_logger().info(f"[LogQuery] 요청 수신: {query_type}")

        if query_type not in self.query_map:
            response.json_result = json.dumps({ "error": f"Unknown query_type: {query_type}" })
            return response

        try:
            result = self.query_map[query_type]()
            response.json_result = json.dumps(result, default=str)
        except Exception as e:
            self.get_logger().error(f"[LogQuery] 쿼리 처리 중 오류: {e}")
            response.json_result = json.dumps({ "error": str(e) })

        return response

def main(args=None):
    rclpy.init(args=args)
    node = LogQueryService()
    rclpy.spin(node)
    rclpy.shutdown()
