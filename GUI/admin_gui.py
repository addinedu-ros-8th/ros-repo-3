from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QTableWidget, QTableWidgetItem,
                             QTextEdit, QGridLayout)
import sys

class AdminGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Admin GUI Dashboard")
        self.resize(1200, 800)
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.create_dashboard()
        self.create_robot_management()
        self.create_request_control()
        self.create_log_viewer()

    def create_dashboard(self):
        label = QLabel("Admin Dashboard (SR_05, SR_01, SR_02, SR_09)")
        label.setStyleSheet("font-weight: bold; font-size: 20px")
        self.layout.addWidget(label)

    def create_robot_management(self):
        robot_label = QLabel("로봇 관리 및 모니터링")
        robot_label.setStyleSheet("font-weight: bold; font-size: 20px")
        self.layout.addWidget(robot_label)

        robot_table = QTableWidget(5, 4)
        robot_table.setHorizontalHeaderLabels(["로봇 ID", "상태", "작업 ID", "배터리"])
        for i in range(5):
            for j in range(4):
                robot_table.setItem(i, j, QTableWidgetItem("-"))
        self.layout.addWidget(robot_table)

    def create_request_control(self):
        request_label = QLabel("요청 현황 및 수동 제어 (SR_06, SR_08, SR_10, SR_12)")
        request_label.setStyleSheet("font-weight: bold; font-size: 20px")
        self.layout.addWidget(request_label)

        request_table = QTableWidget(5, 3)
        request_table.setHorizontalHeaderLabels(["요청 ID", "상태", "수정"])
        self.layout.addWidget(request_table)

    def create_log_viewer(self):
        log_label = QLabel("로그 확인 (SR_14)")
        log_label.setStyleSheet("font-weight: bold; font-size: 20px")
        self.layout.addWidget(log_label)

        log_text = QTextEdit()
        log_text.setPlaceholderText("로봇 및 작업 이벤트 로그...")
        self.layout.addWidget(log_text)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = AdminGUI()
    window.show()
    sys.exit(app.exec_())