from PyQt6.QtCore import QObject, pyqtSignal, QTimer
import rclpy
from rclpy.node import Node
from shared_interfaces.msg import RoscarRegister, RoscarInfo
from shared_interfaces.srv import LogQuery, Login
from geometry_msgs.msg import PoseStamped

class ManagerROS(QObject):
    pose_received = pyqtSignal(object)  # PoseStamped
    roscar_registered = pyqtSignal(object)  # RoscarRegister
    log_query_response = pyqtSignal(str, list)  # query_type, data (json)
    login_response = pyqtSignal(bool, str, int)  # success, role, user_id

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('monitor_panel')

        # ROS 통신 설정
        self.node.create_subscription(PoseStamped, '/main_server/roscar_pose', self._pose_cb, 10)
        self.node.create_subscription(RoscarRegister, '/roscar/register', self._register_cb, 10)
        self.roscar_info_publisher = self.node.create_publisher(RoscarInfo, '/roscar/access', 10)
        self.log_query_client = self.node.create_client(LogQuery, '/log/request/query')
        self.login_client = self.node.create_client(Login, '/user/manager/login')
        
        # ROS spin 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))

    def start(self):
        self.timer.start(50)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.timer.stop()

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
            import json
            self.log_query_response.emit(query_type, json.loads(result.json_result))
        except Exception as e:
            print(f"[ROS] 로그 응답 실패: {e}")

    def _handle_login_response(self, future):
        try:
            res = future.result()
            self.login_response.emit(res.success, res.role, res.user_id)
        except Exception as e:
            print(f"[ROS] 로그인 응답 실패: {e}")
            self.login_response.emit(False, "", 0)

    def request_login(self, username, password):
        if not self.login_client.wait_for_service(timeout_sec=1.0):
            print("[ROS] 로그인 서비스 대기 실패")
            self.login_response.emit(False, "", 0)
            return
        req = Login.Request()
        req.username = username
        req.password = password
        future = self.login_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_login_response(f))
