import sys

from PyQt6.QtWidgets import QApplication
from gui.manager.ros_interface.manager_ros_interface import ManagerROS
from gui.manager.view.manager_login import ManagerLoginWindow

def main():
    app = QApplication(sys.argv)

    # ROS 인터페이스는 로그인 창과 대시보드에서 공유
    manager_ros = ManagerROS()
    manager_ros.start()

    # 로그인 창 실행
    login_window = ManagerLoginWindow(manager_ros)
    login_window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
