import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from worker_gui.dashboard_panel import WorkerDashboard
from worker_gui.request_table import TaskRequestTable
from worker_gui.theme import apply_worker_theme


class WorkerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roskatsu Worker Dashboard")
        self.setGeometry(100, 100, 1200, 800)

        apply_worker_theme(self)

        central = QWidget(self)
        main_layout = QVBoxLayout()

        # 상단: 로봇 오버뷰 + 요청 폼
        self.dashboard = WorkerDashboard()
        main_layout.addWidget(self.dashboard)

        # 하단: 작업 요청 테이블
        self.table = TaskRequestTable()
        main_layout.addWidget(self.table)

        # 요청 제출 시 테이블 업데이트 연결
        self.dashboard.task_submitted.connect(self.table.add_task)

        central.setLayout(main_layout)
        self.setCentralWidget(central)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WorkerGUI()
    window.show()
    sys.exit(app.exec_())