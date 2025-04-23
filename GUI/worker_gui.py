from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QGridLayout, QTableWidget, QTableWidgetItem)
import sys

class WorkerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Worker GUI Dashboard")
        self.resize(1000, 700)
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.create_dashboard()
        self.create_task_request()

    def create_dashboard(self):
        dashboard_label = QLabel("Dashboard")
        dashboard_label.setStyleSheet("font-weight: bold; font-size: 20px")

        inventory_status = QLabel("실시간 재고 파악 (SR_05)")
        robot_status = QLabel("로봇 상태 보기 (SR_11, SR_10, SR_13, SR_15)")
        request_history = QLabel("요청 이력 조회 (SR_07, SR_08)")

        self.layout.addWidget(dashboard_label)
        self.layout.addWidget(inventory_status)
        self.layout.addWidget(robot_status)
        self.layout.addWidget(request_history)

    def create_task_request(self):
        task_label = QLabel("작업 요청 입력 (SR_06, SR_10)")
        task_label.setStyleSheet("font-weight: bold; font-size: 20px")
        self.layout.addWidget(task_label)

        form_layout = QGridLayout()
        form_layout.addWidget(QLabel("출발지:"), 0, 0)
        form_layout.addWidget(QComboBox(), 0, 1)
        form_layout.addWidget(QLabel("도착지:"), 1, 0)
        form_layout.addWidget(QComboBox(), 1, 1)
        form_layout.addWidget(QLabel("무게:"), 2, 0)
        form_layout.addWidget(QComboBox(), 2, 1)
        form_layout.addWidget(QLabel("중요도:"), 3, 0)
        form_layout.addWidget(QComboBox(), 3, 1)

        submit_button = QPushButton("요청 제출")
        form_layout.addWidget(submit_button, 4, 0, 1, 2)
        self.layout.addLayout(form_layout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WorkerGUI()
    window.show()
    sys.exit(app.exec_())