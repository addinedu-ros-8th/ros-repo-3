from shared_interfaces.srv import Login
from rclpy.node import Node

class ManagerLoginService(Node):
    def __init__(self, main_ctl_service):
        super().__init__('manager_login_service')
        self.main_ctl_service = main_ctl_service
        self.srv = self.create_service(Login, '/user/manager/login', self.handle_login)

    def handle_login(self, request, response):
        try:
            result = self.main_ctl_service.run_query("get_user_by_name", request.user_name)

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
                response.user_id = 0
                response.role = ""
                self.get_logger().warning("[로그인 실패] 사용자 정보 불일치")
                self.get_logger().warning(f"[로그인 실패] user_name={request.user_name}, password={request.password}")
        except Exception as e:
            self.get_logger().error(f"[LoginService] 쿼리 처리 중 오류: {e}")
            response.success = False
            response.user_id = 0
            response.role = ""

        return response
