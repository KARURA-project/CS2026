from PySide6.QtWidgets import QMainWindow
from PySide6.QtCore import Slot
from .MobilityScreen import Ui_MainWindow

class MobilityMainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Build the designer UI directly onto this QMainWindow
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # --- State Variables for Persistence ---
        self._last_batt_rem = None
        self._last_batt_volt = None
        self._last_batt_pow = None
        self._last_motor_speeds = [0.0, 0.0, 0.0, 0.0]
        self._last_motor_angles = [0.0, 0.0, 0.0, 0.0]

        # Optional: expose commonly-used widgets for convenience
        if hasattr(self.ui, "maincameravideo"):
            self.maincameravideo = self.ui.maincameravideo

    def connect_signals(self, bridge):
        self.bridge = bridge
        bridge.battery_data_signal.connect(self._update_battery_ui)
        bridge.battery_voltage_data_signal.connect(self._update_battery_voltage)
        bridge.battery_power_data_signal.connect(self._update_battery_power)
        bridge.actual_rads_signal.connect(self._update_motor_angular_speed)
        bridge.actual_angle_signal.connect(self._update_motor_angle)
        bridge.roll_pitch_yaw_signal.connect(self._update_row_pitch_yaw)

    @Slot(object) 
    def _update_battery_ui(self, msg):
        if hasattr(self.ui, "BatteryBox"):
            self._last_batt_rem = float(msg)
            # Pass all three so the others don't revert to "NULL"
            self.ui.BatteryBox.set_values(
                voltage=self._last_batt_volt, 
                power=self._last_batt_pow, 
                remaining=self._last_batt_rem
            )
        else:
            print("[WARN] Battery widget not found in UI")

    @Slot(object)
    def _update_battery_voltage(self, msg):
        if hasattr(self.ui, "BatteryBox"):
            self._last_batt_volt = float(msg)
            self.ui.BatteryBox.set_values(
                voltage=self._last_batt_volt, 
                power=self._last_batt_pow, 
                remaining=self._last_batt_rem
            )

    @Slot(object)
    def _update_battery_power(self, msg):
        if hasattr(self.ui, "BatteryBox"):
            self._last_batt_pow = float(msg)
            self.ui.BatteryBox.set_values(
                voltage=self._last_batt_volt, 
                power=self._last_batt_pow, 
                remaining=self._last_batt_rem
            )

    @Slot(object)
    def _update_motor_angular_speed(self, msg):
        if hasattr(self.ui, "MotorPanel"):
            self._last_motor_speeds = list(msg.data)
            # Pass both current speed and the stored angles
            self.ui.MotorPanel.update_from_ros(
                speeds=self._last_motor_speeds,
                angles_deg=self._last_motor_angles
            )

    @Slot(object)
    def _update_motor_angle(self, msg):
        if hasattr(self.ui, "MotorPanel"):
            self._last_motor_angles = list(msg.data)
            # Pass both current angles and the stored speeds
            self.ui.MotorPanel.update_from_ros(
                speeds=self._last_motor_speeds,
                angles_deg=self._last_motor_angles
            )
    
    @Slot(object)
    def _update_row_pitch_yaw(self, msg):
        if hasattr(self.ui, "MotorPanel"):
            # logic here if needed
            pass