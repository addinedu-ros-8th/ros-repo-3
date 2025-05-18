# server/main_server/main_service/ros_interface/service/log_query_service.py

import rclpy
from rclpy.node import Node
from shared_interfaces.srv import LogQuery
from shared_interfaces.msg import LogEvent
from builtin_interfaces.msg import Time
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.query import MainServiceQuery

class LogQueryService(Node):
    def __init__(self, roscars_session, roscars_log_session):
        super().__init__('log_query_service')
        self.srv = self.create_service(LogQuery, '/log/request/query', self.handle_query)
        self.db = MainServiceQuery(roscars_session, roscars_log_session)


    def handle_query(self, request, response):
        start = self._to_datetime(request.start_time)
        end = self._to_datetime(request.end_time)
        logs = self.db.query_logs(
            log_type=request.log_type,
            start_time=start,
            end_time=end,
            keyword=request.keyword,
            event_codes=list(request.event_filter)
        )

        response.count = len(logs)
        response.logs = [self._to_log_event(msg) for msg in logs]
        return response

    def _to_datetime(self, ros_time: Time):
        from datetime import datetime
        return datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1e9)

    def _to_log_event(self, db_row):
        msg = LogEvent()
        msg.event_id = db_row.event_id
        msg.event_type = db_row.event_type.value if hasattr(db_row.event_type, 'value') else int(db_row.event_type)
        msg.event_data = db_row.message

        ts = Time()
        unix = db_row.timestamp.timestamp()
        ts.sec = int(unix)
        ts.nanosec = int((unix - int(unix)) * 1e9)
        msg.stamp = ts
        return msg

def main(args=None):
    rclpy.init(args=args)
    db = DatabaseManager()
    node = LogQueryService(
        db.get_session("roscars"),
        db.get_session("roscars_log")
    )
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    print("Log Query Service")
    main()
