import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from widgets import MainCameraPanel, MotorInfoPanel
from MobilityScreen import Ui_MainWindow
from PySide6.QtCore import QTimer
import time
import random

app = QApplication(sys.argv)

class MobilityScreen(QMainWindow): 
    def __init__(self, parent=None):
        super().__init__(parent)

        # Sets the UI to use the one made by designer
        self.ui = Ui_MainWindow()
        # self is now a QMainWindow, which has setCentralWidget()
        self.ui.setupUi(self)

class MobilityMainWindow(QMainWindow):
    def __init__(self):
            super().__init__()

            # Set the central widget to the custom MobilityScreen
            self.central = MobilityScreen()
            self.setCentralWidget(self.central)
            self.setStyleSheet("background-color: gray;")
mobility_window = MobilityMainWindow()
mobility_window.setWindowTitle("Mobility")
mobility_window.resize(600, 400)

mobility_window.show()

sys.exit(app.exec())