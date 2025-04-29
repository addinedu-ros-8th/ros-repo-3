import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from viewer.theme import apply_theme
from .dashboard_panel import MonitorPanel

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Manager Dashboard")
        self.setGeometry(150, 150, 1400, 900)

        apply_theme(self)

        central = QWidget()
        layout = QVBoxLayout()

        self.monitor_panel = MonitorPanel()
        layout.addWidget(self.monitor_panel)

        central.setLayout(layout)
        self.setCentralWidget(central)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
