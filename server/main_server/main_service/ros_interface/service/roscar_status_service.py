from rclpy.node import Node
from shared_interfaces.srv import QueryRoscarStatus
from shared_interfaces.msg import RoscarStatus

class QueryRoscarStatusService(Node):
    def __init__(self, main_ctl_service):
        super().__init__('query_roscar_status_service')
        self.main_ctl_service = main_ctl_service
        self.srv = self.create_service(
            QueryRoscarStatus,
            'query_roscar_status',
            self.handle_query
        )

    def handle_query(self, request, response):
        try:
            rows = self.main_ctl_service.run_query("get_available_roscars")  # 또는 get_all_roscars

            for r in rows:
                rs = RoscarStatus()
                rs.roscar_namespace    = r.roscar_namespace
                rs.battery_percentage  = r.battery_percentage
                rs.operational_status  = r.operational_status.name
                response.ros_cars.append(rs)
        except Exception as e:
            self.get_logger().error(f"[RoscarStatus] 조회 실패: {e}")
        return response
