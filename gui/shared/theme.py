def apply_theme(widget):
    widget.setStyleSheet("""
        /* 전체 배경과 기본 텍스트 */
        QMainWindow, QDialog, QWidget {
            background-color: white;
            color: black;
        }

        /* 제목 레이블 */
        QLabel#titleLabel {
            font-size: 20px;
            font-weight: bold;
            color: black;
        }

        /* 그룹박스 */
        QGroupBox {
            background-color: white;
            border: 1px solid black;
            border-radius: 6px;
            margin-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            font-weight: bold;
            color: black;
        }

        /* 버튼 */
        QPushButton {
            background-color: black;
            color: white;
            border-radius: 5px;
            padding: 6px 12px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #333333;
        }

        /* 테이블 위젯 */
        QTableWidget {
            background-color: white;
            color: black;
            gridline-color: black;
        }
        QHeaderView::section {
            background-color: black;
            color: white;
            font-weight: bold;
            padding: 6px;
            border: 1px solid black;
        }
    """)
