#!/usr/bin/env python3
# roscar_status_client.py
# DB의 RosCars 테이블을 주기적으로 조회하는 인터페이스

from PyQt6.QtCore import QObject, pyqtSignal, QTimer
import rclpy
from rclpy.node import Node
from shared_interfaces.srv import QueryRoscarStatus

class RoscarStatusClient(QObject):
    """
    QueryRoscarStatus 서비스를 주기 호출해
    [{'roscar_namespace':..., 'battery_percentage':..., 'operational_status':...}, …]
    형태의 리스트를 roscar_status_response 시그널로 방출합니다.
    """
    roscar_status_response = pyqtSignal(list)

    def __init__(self, poll_interval_ms=5000, spin_interval_ms=50):
        super().__init__()
        # rclpy.init()은 외부(ManagerROS)에서 이미 호출되었다고 가정
        self.node = Node('roscar_status_client')

        # 서비스 클라이언트 생성
        self.client = self.node.create_client(
            QueryRoscarStatus,
            '/query_roscar_status'
        )

        # spin_once() 주기 타이머
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(spin_interval_ms)

        # 서비스 호출 주기 타이머
        self.call_timer = QTimer(self)
        self.call_timer.timeout.connect(self.request_status)
        self.call_timer.start(poll_interval_ms)

    def _spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def request_status(self):
        # 서비스가 준비되지 않았다면 아무 동작하지 않음
        if not self.client.wait_for_service(timeout_sec=0.1):
            return
        # 요청 전송
        req = QueryRoscarStatus.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self._handle_response)

    def _handle_response(self, future):
        try:
            resp = future.result()
            data = [
                {
                    'roscar_namespace': item.roscar_namespace,
                    'battery_percentage': item.battery_percentage,
                    'operational_status': item.operational_status
                }
                for item in resp.ros_cars
            ]
            # 결과 시그널 발행
            self.roscar_status_response.emit(data)
        except Exception as e:
            self.node.get_logger().error(f'RoscarStatus 응답 오류: {e}')

    def shutdown(self):
        self.spin_timer.stop()
        self.call_timer.stop()
        self.node.destroy_node()
        # rclpy.shutdown()은 외부에서 한 번만 호출하도록 하세요
