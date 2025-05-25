from PyQt6.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from shared_interfaces.msg import RoscarRegister
from shared_interfaces.srv import LogQuery, Login, QueryRoscarStatus
from geometry_msgs.msg import PoseStamped
import json

class ManagerROS(QObject):
    pose_received = pyqtSignal(object)
    roscar_registered = pyqtSignal(object)
    log_query_response = pyqtSignal(str, list)
    login_response = pyqtSignal(bool, str, int)
    roscar_status_response = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.node = Node('manager_gui')

        self.node.create_subscription(PoseStamped, '/main_server/roscar_pose', self._pose_cb, 10)
        self.node.create_subscription(RoscarRegister, '/roscar/register', self._register_cb, 10)
        self.log_query_client = self.node.create_client(LogQuery, '/log/request/query')
        self.status_query_client = self.node.create_client(QueryRoscarStatus, '/query_roscar_status')
        self.login_client = self.node.create_client(Login, '/user/manager/login')

    def request_log(self, query_type):
        if not self.log_query_client.wait_for_service(timeout_sec=1.0):
            return
        req = LogQuery.Request()
        req.query_type = query_type
        future = self.log_query_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_log_response(f, query_type))

    def _pose_cb(self, msg: PoseStamped):
        self.pose_received.emit(msg)

    def _register_cb(self, msg: RoscarRegister):
        self.roscar_registered.emit(msg)

    def _handle_log_response(self, future, query_type):
        try:
            result = future.result()
            self.log_query_response.emit(query_type, json.loads(result.json_result))
        except Exception as e:
            self.node.get_logger().error(f"[ROS] 로그 응답 실패: {e}")

    def request_login(self, user_name, password):
        if not self.login_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("[ROS] 로그인 서비스가 응답하지 않습니다.")
            self.login_response.emit(False, "", 0)
            return
        req = Login.Request()
        req.user_name = user_name
        req.password = password
        future = self.login_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_login_response(f))

    def _handle_login_response(self, future):
        try:
            res = future.result()
            self.login_response.emit(res.success, res.role, res.user_id)
            self.node.get_logger().info(f"[ROS] 로그인 응답: 성공={res.success}, 역할={res.role}, 사용자 ID={res.user_id}")
        except Exception as e:
            self.node.get_logger().error(f"[ROS] 로그인 응답 실패: {e}")
            self.login_response.emit(False, "", 0)

    def request_status(self):
        if not self.status_query_client.wait_for_service(timeout_sec=0.1):
            return
        req = QueryRoscarStatus.Request()
        future = self.status_query_client.call_async(req)
        future.add_done_callback(self._handle_status_response)

    def _handle_status_response(self, future):
        try:
            resp = future.result()
            data = [{
                'roscar_namespace': item.roscar_namespace,
                'battery_percentage': item.battery_percentage,
                'operational_status': item.operational_status
            } for item in resp.ros_cars]
            self.roscar_status_response.emit(data)
        except Exception as e:
            self.node.get_logger().error(f"[ROS] 상태 응답 실패: {e}")
