import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt6.QtCore import QObject, pyqtSignal, QThread

class ROSNodeStaff(QObject):
    battery_updated = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._node = None
        self.running = False

    def start_ros_node(self):
        rclpy.init()
        self._node = rclpy.create_node('roskatsu_gui_node')
        self._node.create_subscription(
            String,
            '/robot/status/battery',
            self._battery_callback,
            10
        )
        self.running = True
        while rclpy.ok() and self.running:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_node()
        rclpy.shutdown()

    def stop_ros_node(self):
        self.running = False

    def _battery_callback(self, msg):
        self.battery_updated.emit(msg.data)

class ROSRunner(QThread):
    def __init__(self, ros_staff):
        super().__init__()
        self.ros_staff = ros_staff

    def run(self):
        self.ros_staff.start_ros_node()

    def stop(self):
        self.ros_staff.stop_ros_node()
        self.quit()
        self.wait()
