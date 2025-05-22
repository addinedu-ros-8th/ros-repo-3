import rclpy
from rclpy.node import Node
from shared_interfaces.msg import LogEvent
from builtin_interfaces.msg import Time
import json
import time

class LogEventPublisher(Node):
    def __init__(self):
        super().__init__('log_event_publisher')
        self.publisher = self.create_publisher(LogEvent, '/log/event', 10)
        self.get_logger().info('[LogEventPublisher] 초기화 완료')

    def publish_event(self, event_id: int, event_type: int, event_data: dict):
        msg = LogEvent()
        msg.event_id = event_id
        msg.event_type = event_type
        msg.event_data = json.dumps(event_data)

        now_sec = int(time.time())
        now_nsec = int((time.time() - now_sec) * 1e9)
        msg.stamp = Time(sec=now_sec, nanosec=now_nsec)

        self.publisher.publish(msg)
        self.get_logger().info(f'[LogEventPublisher] 전송 완료 → ID={event_id}, TYPE={event_type}')

def main(args=None):
    rclpy.init(args=args)
    node = LogEventPublisher()

    # 테스트용 publish 호출
    import time
    time.sleep(1.0)  # 노드 초기화 대기
    node.publish_event(
        event_id=1001,
        event_type=0x01,
        event_data={"message": "Test log from publisher"}
    )

    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
