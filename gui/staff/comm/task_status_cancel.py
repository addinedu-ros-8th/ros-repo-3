from PyQt6.QtWidgets import (
    QVBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QPushButton,
    QHBoxLayout,
    QMessageBox,
    QLabel
)
from gui.staff.base_panel import BasePanel

class TaskStatusPanel(BasePanel):
    def __init__(self, tcp_thread, user_id, main_window, parent=None):
        super().__init__(parent)
        self.tcp_client  = tcp_thread
        self.main_window = main_window
        self.user_id     = user_id
        self.delivery_ids = []

        # connect to router signals
        router = self.main_window.router
        router.taskStatusReceived.connect(self.update_task_table)
        router.cancelSuccess.connect(self._on_cancel_success)

        self._init_ui()
        self.load_task_status()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        self.summary_label = QLabel("완료: 0, 진행 중: 0", self)
        layout.addWidget(self.summary_label)

        # 5 columns: ID, 모델명, 상태, 요청자, 버튼
        self.task_table = QTableWidget(0, 5, self)
        self.task_table.setHorizontalHeaderLabels(
            ["작업 ID", "모델명", "상태", "요청자", ""]
        )
        layout.addWidget(self.task_table)

        btn_layout = QHBoxLayout()
        # 인식 버튼
        recog_btn = QPushButton("인식", self)
        recog_btn.clicked.connect(self._go_to_camera)
        btn_layout.addWidget(recog_btn)
        # 전체 취소 버튼
        cancel_all_btn = QPushButton("전체 취소", self)
        cancel_all_btn.clicked.connect(self._cancel_all)
        btn_layout.addWidget(cancel_all_btn)
        btn_layout.addStretch()

        layout.addLayout(btn_layout)

    def load_task_status(self):
        """서버에 TR 요청 보내기"""
        self.tcp_client.send_task_status_request(self.user_id)

    def update_task_table(self, done_count, items):
        """
        items = list of (delivery_id, model_name) for tasks that are IN_PROGRESS
        """
        # 1) 업데이트 summary
        self.summary_label.setText(f"완료: {done_count}, 진행 중: {len(items)}")

        # 2) 완전 초기화
        self.task_table.clearContents()
        self.task_table.setRowCount(0)
        self.delivery_ids = []

        # 3) 새로 채우기
        for did, model_name in items:
            row = self.task_table.rowCount()
            self.task_table.insertRow(row)
            self.delivery_ids.append(did)

            self.task_table.setItem(row, 0, QTableWidgetItem(str(did)))
            self.task_table.setItem(row, 1, QTableWidgetItem(model_name))
            self.task_table.setItem(row, 2, QTableWidgetItem("IN_PROGRESS"))
            self.task_table.setItem(row, 3, QTableWidgetItem(f"사용자 {self.user_id}"))

            btn = QPushButton("취소", self)
            # must capture row in a lambda default arg
            btn.clicked.connect(lambda _, r=row: self._cancel(r))
            self.task_table.setCellWidget(row, 4, btn)

    def _cancel(self, row):
        did = self.delivery_ids[row]
        self.tcp_client.send_cancel_delivery_request(self.user_id, did)

    def _cancel_all(self):
        if not self.delivery_ids:
            QMessageBox.information(self, "알림", "취소할 작업이 없습니다.")
            return
        for i in range(len(self.delivery_ids)):
            self._cancel(i)

    def _on_cancel_success(self, delivery_id):
        """취소 성공 → 팝업 & 테이블 다시 로드"""
        QMessageBox.information(self, "취소 완료", f"배송 ID {delivery_id}이(가) 취소되었습니다.")
        self.load_task_status()

    def _go_to_camera(self):
        self.main_window.go_to_camera()
