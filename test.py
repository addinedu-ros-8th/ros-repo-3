import sys
from viewer.mode_select import RoleChooser
from PyQt5.QtWidgets import QApplication

if __name__ == "__main__":
    app = QApplication(sys.argv)
    chooser = RoleChooser()
    chooser.show()
    sys.exit(app.exec_())
