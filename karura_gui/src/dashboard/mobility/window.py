import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from widgets import MainCameraPanel, MotorInfoPanel
from PySide6.QtCore import QTimer
import time
import random

app = QApplication(sys.argv)

class MobilityMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # self.central = MainCameraPanel()
        # self.setCentralWidget(self.central)

        self.central = MotorInfoPanel()
        self.setCentralWidget(self.central)

        self.central.add_battery()
        self.central.add_battery()
        self.central.add_battery()
        self.central.add_battery()
        self.central.add_battery()
        self.central.add_battery()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(3000)

    def update_ui(self):
        temp_arr = []
        for i in range(0, len(self.central.motors)):
            temp_arr.append({
                "speed": int(random.normalvariate(50, 10)),
                "battery": int(random.normalvariate(60, 5))
            })
        self.central.update_values(temp_arr)

mobility_window = MobilityMainWindow()
mobility_window.setWindowTitle("Mobility")
mobility_window.resize(600, 400)

mobility_window.show()

sys.exit(app.exec())