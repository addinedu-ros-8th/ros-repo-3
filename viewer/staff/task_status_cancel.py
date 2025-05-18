from PyQt6.QtWidgets import QVBoxLayout, QTableWidget, QTableWidgetItem, QPushButton, QHBoxLayout, QMessageBox, QLabel
from viewer.staff.base_panel import BasePanel

class TaskStatusPanel(BasePanel):
    def __init__(self, tcp_thread, user_id, main_window, parent=None):
        super().__init__(parent)
        self.tcp_client = tcp_thread
        self.main_window = main_window
        self.user_id = user_id
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # 요약 정보를 표시할 레이블
        self.summary_label = QLabel(f"완료: 0, 진행 중: 0", self)
        layout.addWidget(self.summary_label)

        # 작업 목록 테이블
        self.task_table = QTableWidget(0, 4, self)
        self.task_table.setHorizontalHeaderLabels(["로봇이름", "상태", "요청자", ""])
        layout.addWidget(self.task_table)

        # 하단 버튼 영역
        btn_layout = QHBoxLayout()

        # 인식 버튼
        recognition_btn = QPushButton("인식", self)
        recognition_btn.clicked.connect(self._go_to_camera_panel)
        btn_layout.addWidget(recognition_btn)

        # 전체 취소 버튼
        cancel_btn = QPushButton("전체 취소", self)
        cancel_btn.clicked.connect(self._cancel_all)
        btn_layout.addWidget(cancel_btn)

        # 오른쪽 정렬
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def load_task_status(self):
        self.tcp_client.send_task_status_request(self.user_id)

    def update_task_table(self, done_count, names):
        # 요약 레이블 업데이트
        self.summary_label.setText(f"완료: {done_count}, 진행 중: {len(names)}")

        # 테이블 초기화 및 행 설정
        self.task_table.setRowCount(len(names))
        for i, name in enumerate(names):
            self.task_table.setItem(i, 0, QTableWidgetItem(name))
            self.task_table.setItem(i, 1, QTableWidgetItem("IN_PROGRESS"))
            self.task_table.setItem(i, 2, QTableWidgetItem(f"사용자 {self.user_id}"))

            btn = QPushButton("취소", self)
            btn.clicked.connect(lambda _, r=i: self._cancel(r))
            self.task_table.setCellWidget(i, 3, btn)

    def _cancel(self, row):
        did = 1000 + row
        self.tcp_client.send_cancel_delivery_request(self.user_id, did)
        QMessageBox.information(self, "취소 요청", f"배송 ID {did} 취소 요청 전송")

    def _cancel_all(self):
        rows = self.task_table.rowCount()
        if rows == 0:
            QMessageBox.information(self, "알림", "취소할 작업이 없음")
            return
        for i in range(rows):
            self._cancel(i)

    def _go_to_camera_panel(self):
        self.main_window.go_to_camera()