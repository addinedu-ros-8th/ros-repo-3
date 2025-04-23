# theme.py
def apply_kaki_theme(widget):
    widget.setStyleSheet("""
        QMainWindow, QDialog, QWidget {
            background-color: #f4f2ec;
        }

        QLabel#titleLabel {
            font-size: 22px;
            font-weight: bold;
            color: #3e3e3e;
        }

        QGroupBox {
            border: 1px solid #c2bca2;
            border-radius: 8px;
            margin-top: 10px;
            background-color: #fffdf6;
        }

        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px 0 3px;
            font-weight: bold;
            color: #5b5a4e;
        }

        QPushButton {
            background-color: #708238;
            color: white;
            border-radius: 6px;
            padding: 8px;
            font-weight: bold;
        }

        QPushButton:hover {
            background-color: #5e6f2a;
        }

        QRadioButton {
            font-size: 14px;
            color: #3e3e3e;
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
