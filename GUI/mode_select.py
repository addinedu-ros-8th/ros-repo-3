import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt
from GUI.admin_gui.main import MainWindow as AdminMainWindow
from GUI.worker_gui.main import WorkerGUI
from GUI.admin_gui.theme import apply_kaki_theme



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

        # Worker 버튼이 위로, 버튼 크기 키움
        worker_btn = QPushButton("Worker GUI")
        worker_btn.setMinimumHeight(60)
        worker_btn.clicked.connect(self.launch_worker_gui)
        layout.addWidget(worker_btn)

        admin_btn = QPushButton("Admin GUI")
        admin_btn.setMinimumHeight(60)
        admin_btn.clicked.connect(self.launch_admin_gui)
        layout.addWidget(admin_btn)

        central.setLayout(layout)
        self.setCentralWidget(central)

    def launch_admin_gui(self):
        self.admin_window = AdminMainWindow()
        self.admin_window.show()
        self.close()

    def launch_worker_gui(self):
        self.worker_window = WorkerGUI()
        self.worker_window.show()
        self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    chooser = RoleChooser()
    chooser.show()
    sys.exit(app.exec_())