from PySide6.QtWidgets import QWidget, QHBoxLayout, QPushButton, QLabel
from PySide6.QtCore import Qt, QTimer

class BottomBarView(QWidget):
    """
    Displays mission time and provides control buttons.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(16)

        self.lbl_mode = QLabel("Mode: IDLE")
        self.lbl_link = QLabel("Link: DISCONNECTED")
        self.lbl_time = QLabel("UTC: --:--:--")

        # stretch in the middle to push time to the right, etc.
        layout.addWidget(self.lbl_mode)
        layout.addWidget(self.lbl_link)
        layout.addStretch()
        layout.addWidget(self.lbl_time)

        # subtle top border so it feels like a bar
        self.setStyleSheet(
            "BottomBar { border-top: 1px solid #555; }"
            "background-color: red;"
        )