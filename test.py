# test.py

import sys
from PyQt6.QtWidgets import QApplication
from viewer.mode_select import RoleChooser

if __name__ == "__main__":
    app = QApplication(sys.argv)
    chooser = RoleChooser()
    chooser.show()
    sys.exit(app.exec())
