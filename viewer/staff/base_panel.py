from PyQt6.QtWidgets import QWidget
from viewer.theme import apply_theme

class BasePanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        apply_theme(self)