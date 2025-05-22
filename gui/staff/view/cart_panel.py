# viewer/staff/cart_panel.py

from PyQt6.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt6.QtCore import Qt
from gui.theme import apply_theme

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
        self.cart_table.setHorizontalHeaderLabels(["모델명", "색상", "사이즈", "랙 위치", "재고 수량", "취소"])
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
        self.cart_table.setRowCount(0)  # 테이블 초기화

        # 캐쉬에서 담은 상품들을 불러옴
        cached_items = self.parent_gui.cache_manager.get_items()

        if cached_items:
            for product in cached_items:
                row_pos = self.cart_table.rowCount()
                self.cart_table.insertRow(row_pos)

                self.cart_table.setItem(row_pos, 0, QTableWidgetItem(product["model"]))
                self.cart_table.setItem(row_pos, 1, QTableWidgetItem(product["color"]))
                self.cart_table.setItem(row_pos, 2, QTableWidgetItem(str(product["size"])))
                self.cart_table.setItem(row_pos, 3, QTableWidgetItem(product["rack"]))
                self.cart_table.setItem(row_pos, 4, QTableWidgetItem(str(product["quantity"])))

                cancel_btn = QPushButton("취소")
                cancel_btn.setFixedHeight(30)
                cancel_btn.clicked.connect(lambda r=row_pos: self.remove_item(r))
                self.cart_table.setCellWidget(row_pos, 5, cancel_btn)
        else:
            print("담은 상품이 없습니다.")

    def remove_item(self, row):
        """특정 행 삭제"""
        # 행이 존재하는지 확인
        if self.cart_table.rowCount() <= row:
            print(f"Row {row}가 존재하지 않습니다.")
            return

        # 모델명으로 상품을 식별
        product = self.cart_table.item(row, 0).text()
        print(f"Row {row} 삭제 요청됨: {product}")

        # 캐시에서 해당 아이템 제거
        cached_items = self.parent_gui.cache_manager.get_items()
        item_to_remove = None

        for item in cached_items:
            if item["model"] == product:
                item_to_remove = item  # 삭제할 아이템 찾음
                break

        if item_to_remove:
            self.parent_gui.cache_manager.remove_item(item_to_remove)  # 캐시에서 삭제
            print(f"장바구니에서 삭제됨: {item_to_remove}")
        else:
            print(f"장바구니에 항목이 없습니다: {product}")

        # UI에서 삭제 후 행 개수 확인
        self.cart_table.removeRow(row)  # UI에서 삭제
        
    def on_request_clicked(self):
        self.parent_gui.go_to_request_wait(status_text="요청중...")

    def on_close_clicked(self):
        print("닫기 버튼 클릭 (Cart)")
        self.parent_gui.go_to_product_info()
