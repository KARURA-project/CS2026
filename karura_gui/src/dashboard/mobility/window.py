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
        bridge.battery_data_signal.connect(self._update_battery_ui)
        bridge.battery_voltage_data_signal.connect(self._update_battery_voltage)
        bridge.battery_power_data_signal.connect(self._update_battery_power)
        bridge.actual_rads_signal.connect(self._update_motor_angular_speed)
        bridge.roll_pitch_yaw_signal.connect(self._update_row_pitch_yaw)

    @Slot(object) 
    def _update_battery_ui(self, msg):
        if hasattr(self.ui, "BatteryBox"):
            percentage = float(msg)
            self.ui.BatteryBox.set_values(remaining=percentage)
        else:
            print("[WARN] Battery widget not found in UI")

    @Slot(object)
    def _update_battery_voltage(self, msg):
        if hasattr(self.ui, "BatteryBox"):
            self.ui.BatteryBox.set_values(voltage=float(msg))

    @Slot(object)
    def _update_battery_power(self, msg):
        if hasattr(self.ui, "BatteryBox"):
            self.ui.BatteryBox.set_values(power=float(msg))

    @Slot(object)
    def _update_motor_angular_speed(self, msg):
        if hasattr(self.ui, "MotorPanel"):
            speeds = list(msg.data)

            self.ui.MotorPanel.update_from_ros(speeds=speeds)
    
    @Slot(object)
    def _update_row_pitch_yaw(self, msg):
        if hasattr(self.ui, "MotorPanel"):
            doNothing = 0
    
