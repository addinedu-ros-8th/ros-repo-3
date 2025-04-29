import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt6.QtCore import QObject, pyqtSignal, QThread

class ROSNodeStaff(QObject):
    battery_updated = pyqtSignal(str)  # 배터리 상태 업데이트 시그널

    def __init__(self):
        super().__init__()
        self._node = None
        self.running = False

    def start_ros_node(self):
        rclpy.init(args=None)  # PyQt6와 충돌 방지, args=None 명시
        self._node = rclpy.create_node('staff_gui_node')

        # 배터리 상태 토픽 구독
        self._node.create_subscription(
            String,
            '/robot/status/battery',
            self._battery_callback,
            10
        )

        self.running = True
        while rclpy.ok() and self.running:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        # 종료 시 처리
        if self._node is not None:
            self._node.destroy_node()
        rclpy.shutdown()

    def stop_ros_node(self):
        self.running = False

    def _battery_callback(self, msg):
        self.battery_updated.emit(msg.data)  # 시그널로 데이터 전달

class ROSRunner(QThread):
    def __init__(self, ros_node: ROSNodeStaff):
        super().__init__()
        self.ros_node = ros_node

    def run(self):
        self.ros_node.start_ros_node()

    def stop(self):
        self.ros_node.stop_ros_node()
        self.quit()
        self.wait()
