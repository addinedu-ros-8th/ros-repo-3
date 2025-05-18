from PyQt6.QtWidgets import QVBoxLayout, QTableWidget, QTableWidgetItem, QPushButton, QHBoxLayout
from viewer.staff.base_panel import BasePanel

class ProductInfoPanel(BasePanel):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self.main_window = main_window
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # 상품 정보 테이블
        self.product_table = QTableWidget(1, 5, self)
        self.product_table.setHorizontalHeaderLabels([
            "모델명", "색상", "사이즈", "랙 위치", "재고수량"
        ])
        layout.addWidget(self.product_table)

        # 버튼 영역
        btn_layout = QHBoxLayout()
        buttons = [
            ("담기", self.on_add_to_cart_clicked),
            ("담은목록", self.on_view_cart_clicked),
            ("닫기", self.on_close_clicked),
        ]
        for name, slot in buttons:
            btn = QPushButton(name, self)
            btn.clicked.connect(slot)
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)

    def update_product_info(self, data):
        keys = ["model", "color", "size", "rack", "quantity"]
        for col, key in enumerate(keys):
            self.product_table.setItem(
                0, col, QTableWidgetItem(str(data.get(key, "-")))
            )
        self.current_product = data

    def on_add_to_cart_clicked(self):
        self.main_window.cache_manager.add_item(self.current_product)

    def on_view_cart_clicked(self):
        self.main_window.go_to_cart()

    def on_close_clicked(self):
        self.main_window.go_to_camera()
