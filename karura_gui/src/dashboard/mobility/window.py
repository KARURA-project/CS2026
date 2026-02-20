from PySide6.QtWidgets import QMainWindow
from PySide6.QtCore import Slot
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

        # self.setStyleSheet("background-color: gray;")

    def connect_signals(self, bridge):
        self.bridge = bridge
        # This will now work because self.battery_status is defined
        bridge.battery_data_signal.connect(self._update_battery_ui)

    @Slot(object) 
    def _update_battery_ui(self, msg):
        if hasattr(self.ui, "BatteryBox"):
            percentage = float(msg)
            
            self.ui.BatteryBox.set_values(remaining=percentage)
        else:
            print("[WARN] Battery widget not found in UI")