# viewer/staff/product_info_panel.py

from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt
from viewer.theme import apply_theme

class ProductInfoPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent
        apply_theme(self)

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # 상품정보 테이블
        self.product_table = QTableWidget(1, 5)  # 1행 5열
        self.product_table.setHorizontalHeaderLabels(["모델명", "색상", "사이즈", "랙 위치", "재고수량"])
        self.product_table.horizontalHeader().setStretchLastSection(True)
        self.product_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        main_layout.addWidget(self.product_table)

        # 버튼 영역
        button_layout = QHBoxLayout()
        self.btn_request = QPushButton("요청")
        self.btn_add_to_cart = QPushButton("담기 (장바구니)")
        self.btn_view_cart = QPushButton("담은목록")
        self.btn_close = QPushButton("닫기")

        button_layout.addWidget(self.btn_request)
        button_layout.addWidget(self.btn_add_to_cart)
        button_layout.addWidget(self.btn_view_cart)
        button_layout.addWidget(self.btn_close)

        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

        # 버튼 연결
        self.btn_request.clicked.connect(self.on_request_clicked)
        self.btn_add_to_cart.clicked.connect(self.on_add_to_cart_clicked)
        self.btn_close.clicked.connect(self.on_close_clicked)
        self.btn_view_cart.clicked.connect(self.on_view_cart_clicked)

    def update_product_info(self, product_data):
        """QR 인식된 상품 정보 표시"""
        self.product_table.setItem(0, 0, QTableWidgetItem(product_data.get("model", "-")))
        self.product_table.setItem(0, 1, QTableWidgetItem(product_data.get("color", "-")))
        self.product_table.setItem(0, 2, QTableWidgetItem(str(product_data.get("size", "-"))))
        self.product_table.setItem(0, 3, QTableWidgetItem(product_data.get("rack", "-")))
        self.product_table.setItem(0, 4, QTableWidgetItem(str(product_data.get("quantity", "-"))))

        self.current_product = product_data  # 현재 상품 저장

    def on_request_clicked(self):
        self.parent_gui.go_to_request_wait(status_text="요청중...")

    def on_add_to_cart_clicked(self):
        print("담기 버튼 클릭됨 (ProductInfoPanel)")

        if hasattr(self.parent_gui, "cache_manager"):
            self.parent_gui.cache_manager.add_item(self.current_product)  # 캐쉬 저장
            self.parent_gui.cart_panel.load_cart_items()  # 담은 목록 화면 갱신
        else:
            print("[경고] cache_manager가 설정되지 않았습니다.")

    def on_close_clicked(self):
        self.parent_gui.go_to_camera()

    def on_view_cart_clicked(self):
        self.parent_gui.go_to_cart()
