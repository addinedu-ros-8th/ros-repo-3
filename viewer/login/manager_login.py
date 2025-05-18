import sys
import os
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic

# 현재 Python 파일 위치 기준으로 .ui 경로 설정
base_dir = os.path.dirname(__file__)
ui_path = os.path.join(base_dir, "manager_login.ui")

# UI 파일 로드
from_class = uic.loadUiType(ui_path)[0]

class windowsClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Manager Login Display")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = windowsClass()
    myWindows.show()
    sys.exit(app.exec())
