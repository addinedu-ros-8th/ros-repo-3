# main.py
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QMessageBox
from viewer.theme import apply_theme
from viewer.manager.manager.dashboard_panel import MonitorPanel

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Manager Dashboard")
        self.setGeometry(150, 150, 1400, 900)
        apply_theme(self)

        central = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(MonitorPanel())
        central.setLayout(layout)
        self.setCentralWidget(central)

    def show_message(self, title, message):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Icon.Information)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
