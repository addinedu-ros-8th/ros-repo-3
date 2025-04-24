# theme.py
def apply_worker_theme(widget):
    widget.setStyleSheet("""
        QMainWindow, QDialog, QWidget {
            background-color: #f4f2ec;
        }
        QLabel#titleLabel {
            font-size: 20px;
            font-weight: bold;
            color: #3e3e3e;
        }
        QGroupBox {
            background-color: #fffdf6;
            border: 1px solid #c2bca2;
            border-radius: 6px;
            margin-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            font-weight: bold;
            color: #5b5a4e;
        }
        QPushButton {
            background-color: #708238;
            color: white;
            border-radius: 5px;
            padding: 6px 12px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #5e6f2a;
        }
        QTableWidget {
            background-color: #fffdf6;
            color: #3e3e3e;
            gridline-color: #a89f7d;
        }
        QHeaderView::section {
            background-color: #d2c8a9;
            color: #3e3e3e;
            font-weight: bold;
            padding: 6px;
            border: 1px solid #c2bca2;
        }
    """)