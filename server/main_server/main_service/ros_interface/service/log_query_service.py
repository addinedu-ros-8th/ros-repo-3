from rclpy.node import Node
import json

from shared_interfaces.srv import LogQuery

class LogQueryService(Node):
    def __init__(self, main_ctl_service):
        super().__init__('log_query_service')
        self.main_ctl_service = main_ctl_service
        self.srv = self.create_service(LogQuery, '/log/request/query', self.handle_query)

        self.query_map = {
            "delivery": lambda: self.main_ctl_service.run_query("get_all_delivery_logs"),
            "task": lambda: self.main_ctl_service.run_query("get_task_event_history"),
            "roscar": lambda: self.main_ctl_service.run_query("get_roscar_event_detail"),
            "precision_stop": lambda: self.main_ctl_service.run_query("get_precision_stop_result"),
            "trajectory": lambda: self.main_ctl_service.run_query("get_roscar_trajectory"),
            "driving_event": lambda: self.main_ctl_service.run_query("get_roscar_driving_event_log"),
            "sensor_fusion": lambda: self.main_ctl_service.run_query("get_sensor_for_training"),
            "control_command": lambda: self.main_ctl_service.run_query("get_control_command_log"),
            "filesystem": lambda: self.main_ctl_service.run_query("get_filesystem_log"),
            "rack_sensor": lambda: self.main_ctl_service.run_query("get_rack_sensor_log")
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
