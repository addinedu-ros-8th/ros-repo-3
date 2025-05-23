import rclpy
import json
from shared_interfaces.srv import Login
from rclpy.node import Node
from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.query import MainServiceQuery

class ManagerLoginService(Node):
    def __init__(self):
        super().__init__('manager_login_service')
        self.srv = self.create_service(Login, '/user/manager/login', self.handle_login)

        db = DatabaseManager()
        self.query = MainServiceQuery(
            db.get_session("roscars"),
            db.get_session("roscars_log")
        )

    def handle_login(self, request, response):
        try:
            result = self.query.get_user_by_name(request.username)
            
            if (
                result
                and result.check_password(request.password)
                and result.user_role.value == "MANAGER"
            ):
                response.success = True
                response.user_id = result.user_id
                response.role = result.user_role.value
                self.get_logger().info(f"[로그인 성공] user_id={result.user_id}, role={result.user_role}")
            else:
                response.success = False
                response.role = ""
                response.user_id = 0
                self.get_logger().info("[❌ 로그인 실패] 사용자 정보 불일치")
        except Exception as e:
            self.get_logger().error(f"[LoginService] 쿼리 처리 중 오류: {e}")
            response.success = False
            response.role = ""
            response.user_id = 0

        return response