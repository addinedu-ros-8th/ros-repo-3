import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QMessageBox
from viewer.theme import apply_theme
from .dashboard_panel import MonitorPanel  # MonitorPanel만 포함

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Manager Dashboard")
        self.setGeometry(150, 150, 1400, 900)

        apply_theme(self)

        central = QWidget()
        layout = QVBoxLayout()

        # Monitor Panel (로봇 추가 기능은 MonitorPanel에서 처리)
        self.monitor_panel = MonitorPanel()
        layout.addWidget(self.monitor_panel)

        central.setLayout(layout)
        self.setCentralWidget(central)

    def show_message(self, title, message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Icon.Information)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
