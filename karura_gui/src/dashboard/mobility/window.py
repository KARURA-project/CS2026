from PySide6.QtWidgets import QMainWindow
from .MobilityScreen import Ui_MainWindow


class MobilityMainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Build the designer UI directly onto this QMainWindow
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Optional: expose commonly-used widgets for convenience
        if hasattr(self.ui, "maincameravideo"):
            self.maincameravideo = self.ui.maincameravideo

        self.setStyleSheet("background-color: gray;")

    def connect_signals(self, bridge):
        self.bridge = bridge
