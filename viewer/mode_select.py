import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt
from viewer.manager.main import MainWindow as ManagerMainWindow
from viewer.staff.main import StaffGUI
from viewer.manager.theme import apply_kaki_theme



class RoleChooser(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Role")
        self.setGeometry(200, 200, 400, 300)
        apply_kaki_theme(self)
        self._init_ui()

    def _init_ui(self):
        central = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Choose Interface")
        label.setObjectName("titleLabel")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        # Staff 버튼이 위로, 버튼 크기 키움
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
    sys.exit(app.exec_())