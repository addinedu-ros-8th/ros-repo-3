import rclpy
from rclpy.node import Node
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.models.roscars_models import RosCars
from shared_interfaces.srv import QueryRoscarStatus
from shared_interfaces.msg import RoscarStatus

class QueryRoscarStatusService(Node):
    def __init__(self):
        super().__init__('query_roscar_status_service')
        # 서비스 서버 생성
        self.srv = self.create_service(
            QueryRoscarStatus,
            'query_roscar_status',
            self.handle_query
        )
        # DB 매니저 준비 (handle_query에서 새 세션 사용)
        self.dbm = DatabaseManager()

    def handle_query(self, request, response):
        # 매 호출마다 새로운 세션 생성하여 최신 데이터 조회
        session = self.dbm.get_session('roscars')
        try:
            rows = session.query(RosCars).all()
            for r in rows:
                rs = RoscarStatus()
                rs.roscar_namespace    = r.roscar_namespace
                rs.battery_percentage  = r.battery_percentage
                rs.operational_status  = r.operational_status.name
                response.ros_cars.append(rs)
        finally:
            session.close()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = QueryRoscarStatusService()
    rclpy.spin(node)
    rclpy.shutdown()
