import sys
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from PyQt6.QtWidgets import QApplication

from gui.manager.ros_interface.manager_ros_interface import ManagerROS
from gui.manager.view.manager_login import ManagerLoginWindow

def main():
    rclpy.init()

    # Qt 앱 실행
    app = QApplication(sys.argv)

    # ROS Node
    manager_ros = ManagerROS()

    # ROS Executor Thread
    executor = MultiThreadedExecutor()
    executor.add_node(manager_ros.node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    login_window = ManagerLoginWindow(manager_ros)
    login_window.show()

    exit_code = app.exec()

    # 종료 처리
    executor.shutdown()
    manager_ros.node.destroy_node()
    rclpy.shutdown()
    ros_thread.join(timeout=2)

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
