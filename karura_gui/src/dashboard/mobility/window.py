import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from widgets import MainCameraPanel, MotorInfoPanel

app = QApplication(sys.argv)

class MobilityMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.central = MainCameraPanel()
        self.setCentralWidget(self.central)

        # self.central = MotorInfoPanel()
        # self.setCentralWidget(self.central)

        # self.central.add_battery()
        # self.central.add_battery()
        # self.central.add_battery()
        # self.central.add_battery()
        # self.central.add_battery()
        # self.central.add_battery()

mobility_window = MobilityMainWindow()
mobility_window.setWindowTitle("Mobility")
mobility_window.resize(600, 400)

mobility_window.show()

app.exec()