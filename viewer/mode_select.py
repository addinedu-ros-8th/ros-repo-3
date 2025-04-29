import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'viewer')))  # 경로 추가

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt6.QtCore import Qt
from viewer.manager.main import MainWindow as ManagerMainWindow
from viewer.staff.main import StaffGUI
from viewer.theme import apply_theme  # theme.py 임포트

class RoleChooser(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Role")
        self.setGeometry(200, 200, 400, 300)
        apply_theme(self)  # 테마 적용
        self._init_ui()

    def _init_ui(self):
        central = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Choose Interface")
        label.setObjectName("titleLabel")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(label)

        staff_btn = QPushButton("Staff GUI")
        staff_btn.setMinimumHeight(60)
        staff_btn.clicked.connect(self.launch_staff_gui)
        layout.addWidget(staff_btn)

        manager_btn = QPushButton("Manager GUI")
        manager_btn.setMinimumHeight(60)
        manager_btn.clicked.connect(self.launch_manager_gui)
        layout.addWidget(manager_btn)

        central.setLayout(layout)
        self.setCentralWidget(central)

    def launch_manager_gui(self):
        self.manager_window = ManagerMainWindow()
        self.manager_window.show()
        self.close()

    def launch_staff_gui(self):
        self.staff_window = StaffGUI()
        self.staff_window.show()
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    chooser = RoleChooser()
    chooser.show()
    sys.exit(app.exec())
