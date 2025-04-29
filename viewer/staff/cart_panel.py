# viewer/staff/cart_panel.py

from PyQt6.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt6.QtCore import Qt
from viewer.theme import apply_theme

class CartPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent
        apply_theme(self)

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # 담은목록 테이블
        self.cart_table = QTableWidget(0, 6)  # 다행 6열
        self.cart_table.setHorizontalHeaderLabels(["모델명", "색상", "사이즈", "랙 위치", "수량", "취소"])
        self.cart_table.horizontalHeader().setStretchLastSection(True)
        self.cart_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        main_layout.addWidget(self.cart_table)

        # 하단 버튼
        bottom_layout = QHBoxLayout()
        self.btn_request = QPushButton("전체 요청")
        self.btn_close = QPushButton("닫기")
        bottom_layout.addWidget(self.btn_request)
        bottom_layout.addWidget(self.btn_close)

        main_layout.addLayout(bottom_layout)
        self.setLayout(main_layout)

        # 버튼 이벤트 연결
        self.btn_request.clicked.connect(self.on_request_clicked)
        self.btn_close.clicked.connect(self.on_close_clicked)

    def load_cart_items(self):
        """담긴 상품들을 테이블에 채운다"""
        self.cart_table.setRowCount(0)

        # TODO: 캐쉬 연동 예정, 현재는 테스트 데이터
        dummy_cache = [
            {"model": "Nike Air Max", "color": "Black", "size": "260", "rack": "A1", "quantity": 2},
            {"model": "Adidas UltraBoost", "color": "White", "size": "240", "rack": "B3", "quantity": 1},
        ]

        for product in dummy_cache:
            row_pos = self.cart_table.rowCount()
            self.cart_table.insertRow(row_pos)

            self.cart_table.setItem(row_pos, 0, QTableWidgetItem(product["model"]))
            self.cart_table.setItem(row_pos, 1, QTableWidgetItem(product["color"]))
            self.cart_table.setItem(row_pos, 2, QTableWidgetItem(str(product["size"])))
            self.cart_table.setItem(row_pos, 3, QTableWidgetItem(product["rack"]))
            self.cart_table.setItem(row_pos, 4, QTableWidgetItem(str(product["quantity"])))

            cancel_btn = QPushButton("취소")
            cancel_btn.setFixedHeight(30)
            cancel_btn.clicked.connect(lambda _, r=row_pos: self.remove_item(r))
            self.cart_table.setCellWidget(row_pos, 5, cancel_btn)

    def remove_item(self, row):
        """특정 행 삭제"""
        print(f"Row {row} 삭제 요청됨")
        self.cart_table.removeRow(row)

    def on_request_clicked(self):
        self.parent_gui.go_to_request_wait(status_text="요청중...")

    def on_close_clicked(self):
        self.parent_gui.go_to_camera()
