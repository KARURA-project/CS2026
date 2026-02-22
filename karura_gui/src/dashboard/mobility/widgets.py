from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QPushButton,
    QLabel,
    QSizePolicy,
)
from PySide6.QtCore import (
    QTimer,
    Qt,
    QCoreApplication,
)

from .custom_widgets.MotorInfoBox import Ui_MotorInfoBox
from .custom_widgets.DirectionWidget import Ui_DirectionWidget

import math
import time


# ============================================================
# MotorInfoBox wrapper (logic + null handling)
# ============================================================
class MotorInfoBox(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MotorInfoBox()
        self.ui.setupUi(self)

        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setObjectName("motorInfoCard")

        self.set_null()

    def set_name(self, name: str):
        self.ui.motor_name.setText(QCoreApplication.translate("MotorInfoBox", name, None))

    def set_null(self):
        self.ui.speed_value.setText(QCoreApplication.translate("MotorInfoBox", "NULL", None))
        self.ui.angle_value.setText(QCoreApplication.translate("MotorInfoBox", "NULL", None))

    def set_values(self, speed, angle_deg):
        # Speed
        if speed is None:
            self.ui.speed_value.setText(QCoreApplication.translate("MotorInfoBox", "NULL", None))
        else:
            self.ui.speed_value.setText(QCoreApplication.translate("MotorInfoBox", f"{float(speed):.2f}", None))

        # Angle
        if angle_deg is None:
            self.ui.angle_value.setText(QCoreApplication.translate("MotorInfoBox", "NULL", None))
        else:
            self.ui.angle_value.setText(QCoreApplication.translate("MotorInfoBox", f"{float(angle_deg):.1f}", None))


# ============================================================
# Battery status box (single rover battery)
# ============================================================
class BatteryStatusWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Make QSS backgrounds actually paint on this widget
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setObjectName("BatteryStatusWidget")

        root = QVBoxLayout(self)
        # tighter top padding to remove empty space above title
        root.setContentsMargins(14, 10, 14, 12)
        root.setSpacing(10)

        # Title
        self.title = QLabel("Battery", self)
        self.title.setObjectName("batteryTitle")
        self.title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        root.addWidget(self.title)

        # Rows container
        rows = QVBoxLayout()
        rows.setContentsMargins(0, 0, 0, 0)
        rows.setSpacing(8)

        def mk_row(label_text: str, unit_text: str, value_object_name: str):
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(10)

            lbl = QLabel(label_text, self)
            lbl.setObjectName("batteryKey")
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            val = QLabel("NULL", self)
            val.setObjectName(value_object_name)
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            unit = QLabel(unit_text, self)
            unit.setObjectName("batteryUnit")
            unit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            row.addWidget(lbl, 1)
            row.addWidget(val, 0)
            row.addWidget(unit, 0)

            return row, val

        r1, self.voltageValue = mk_row("Voltage:", "[V]", "batteryVoltageValue")
        r2, self.powerValue   = mk_row("Power:",   "[W]", "batteryPowerValue")
        r3, self.remainValue  = mk_row("Remaining:", "[%]", "batteryRemainValue")

        rows.addLayout(r1)
        rows.addLayout(r2)
        rows.addLayout(r3)

        root.addLayout(rows)

    # Optional helpers so your ROS subscriber can update these easily
    def set_null(self):
        self.voltageValue.setText("NULL")
        self.powerValue.setText("NULL")
        self.remainValue.setText("NULL")

    def set_values(self, voltage=None, power=None, remaining=None):
        self.voltageValue.setText("NULL" if voltage is None else f"{voltage:.2f}")
        self.powerValue.setText("NULL" if power is None else f"{power:.2f}")
        self.remainValue.setText("NULL" if remaining is None else f"{remaining:.0f}")

# class ControlsHintWidget(QWidget):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.setAttribute(Qt.WA_StyledBackground, True)
#         self.setObjectName("ControlsHintWidget")

#         root = QVBoxLayout(self)
#         root.setContentsMargins(14, 10, 14, 12)
#         root.setSpacing(8)

#         title = QLabel("Controls", self)
#         title.setObjectName("controlsTitle")
#         root.addWidget(title)

#         body = QLabel(self)
#         body.setObjectName("controlsBody")
#         body.setWordWrap(True)
#         body.setText(
#             "Joystick:\n"
#             "  • Left stick: drive\n"
#             "  • Right stick: rotate\n"
#             "  • Bumpers/Triggers: speed ±\n"
#             "Keyboard (teleop_twist_keyboard):\n"
#         )
#         root.addWidget(body, 1)

# ============================================================
# Direction widget (your existing class should stay here)
# ============================================================
class DirectionWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_DirectionWidget()
        self.ui.setupUi(self)

        self.setFocusPolicy(Qt.StrongFocus)

        self.idle_style = self.ui.base_style
        self.active_style = """
            QLabel {
                background-color: #3A1C22;
                color: #F2F2F2;
                border: 1px solid #E05E5E;
                border-radius: 10px;
                font-weight: 900;
                font-size: 18px;
                padding: 6px;
            }
        """

        self._labels = {
            "fwd": self.ui.keyForward,
            "back": self.ui.keyBack,
            "left": self.ui.keyLeft,
            "right": self.ui.keyRight,
            "rotl": self.ui.keyRotateLeft,
            "rotr": self.ui.keyRotateRight,
            "spd_up": (self.ui.keySpeedUpTop, self.ui.keySpeedUpBot),
            "spd_dn": (self.ui.keySpeedDownTop, self.ui.keySpeedDownBot),
        }

        self._pressed_dirs = set()

        self._key_to_dir = {
            Qt.Key_Up: "fwd",
            Qt.Key_Down: "back",
            Qt.Key_Left: "left",
            Qt.Key_Right: "right",
        }

        self._key_to_dir_extra = {
            Qt.Key_Q: "rotl",
            Qt.Key_E: "rotr",
        }

        self._key_speed_up = {Qt.Key_Plus, Qt.Key_Equal}
        self._key_speed_dn = {Qt.Key_Minus, Qt.Key_Underscore}

        self._lin_thresh = 0.05
        self._ang_thresh = 0.05
        self._strafe_thresh = 0.05

        self._apply_all_idle()

    def _set_label_active(self, lbl, active: bool):
        lbl.setStyleSheet(self.active_style if active else self.idle_style)

    def _apply_all_idle(self):
        for v in self._labels.values():
            if isinstance(v, tuple):
                for lbl in v:
                    self._set_label_active(lbl, False)
            else:
                self._set_label_active(v, False)

    def _set_dir(self, name: str, active: bool):
        v = self._labels.get(name)
        if v is None:
            return
        if isinstance(v, tuple):
            for lbl in v:
                self._set_label_active(lbl, active)
        else:
            self._set_label_active(v, active)

    def clear(self):
        self._pressed_dirs.clear()
        self._apply_all_idle()

    def set_twist(self, linear_x: float, angular_z: float, strafe_y: float = 0.0):
        for name in ("fwd", "back", "left", "right", "rotl", "rotr"):
            self._set_dir(name, False)

        if linear_x > self._lin_thresh:
            self._set_dir("fwd", True)
        elif linear_x < -self._lin_thresh:
            self._set_dir("back", True)

        if strafe_y > self._strafe_thresh:
            self._set_dir("left", True)
        elif strafe_y < -self._strafe_thresh:
            self._set_dir("right", True)

        if angular_z > self._ang_thresh:
            self._set_dir("rotl", True)
        elif angular_z < -self._ang_thresh:
            self._set_dir("rotr", True)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()

        if key in self._key_to_dir:
            d = self._key_to_dir[key]
            self._pressed_dirs.add(d)
            self._set_dir(d, True)
            return

        if key in self._key_to_dir_extra:
            d = self._key_to_dir_extra[key]
            self._pressed_dirs.add(d)
            self._set_dir(d, True)
            return

        if key in self._key_speed_up:
            self._set_dir("spd_up", True)
            return

        if key in self._key_speed_dn:
            self._set_dir("spd_dn", True)
            return

        return super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()

        if key in self._key_to_dir:
            d = self._key_to_dir[key]
            if d in self._pressed_dirs:
                self._pressed_dirs.remove(d)
            self._set_dir(d, False)
            return

        if key in self._key_to_dir_extra:
            d = self._key_to_dir_extra[key]
            if d in self._pressed_dirs:
                self._pressed_dirs.remove(d)
            self._set_dir(d, False)
            return

        if key in self._key_speed_up:
            self._set_dir("spd_up", False)
            return

        if key in self._key_speed_dn:
            self._set_dir("spd_dn", False)
            return

        return super().keyReleaseEvent(event)


class HelperBox(QWidget):
    def __init__(self, title_text: str, parent=None):
        super().__init__(parent)

        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setObjectName("HelperBox")
        
        self.setMinimumHeight(130)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 10, 14, 12)
        root.setSpacing(8)

        self.title = QLabel(self, text=(
            "<div style='font-size: 16px'>Controls & Information<\div>"
        ))
        self.title.setObjectName("helperBoxTitle") # Match batteryTitle QSS if preferred
        self.title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        root.addWidget(self.title)

        self.body = QLabel(self, text=(
            "<div style='font-size: 12px; line-height: 140%; color: #F2F2F2;'>"
            "<b>Left:</b> &larr;<br>"
            "<b>Right:</b> &rarr;<br>"
            "<b>Up:</b> &uarr;<br>"
            "<b>Down:</b> &darr;<br>"
            "<b>Rotate CCW:</b> Q<br>"
            "<b>Rotate CW:</b> E<br>"
            "<b>Speed Up:</b> +<br>"
            "<b>Slow Down:</b> -"
            "</div>"
        ))
        self.body.setObjectName("statusBoxBody")
        self.body.setTextFormat(Qt.RichText) # Ensures HTML renders correctly
        self.body.setWordWrap(True)
        self.body.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        
        # Add to layout with a stretch factor to push it to the top
        root.addWidget(self.body, 1)

# ============================================================
# Motor panel (2x2 fixed, TL/TR/BL/BR) with speed + angle
# ============================================================
class MotorInfoPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.columns = 2
        self.motors = []

        self.layout = QGridLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(12)

        self._names = [
            "Motor 1 (TL)",
            "Motor 2 (TR)",
            "Motor 3 (BL)",
            "Motor 4 (BR)",
        ]

        for i in range(4):
            box = MotorInfoBox(self)
            box.set_name(self._names[i])
            box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            box.setMinimumHeight(130)

            self.motors.append(box)

            row = i // self.columns
            col = i % self.columns
            self.layout.addWidget(box, row, col)

        self._last_update_ms = None
        self._stale_timeout_ms = 1500

        self._stale_timer = QTimer(self)
        self._stale_timer.setInterval(250)
        self._stale_timer.timeout.connect(self._check_stale)
        self._stale_timer.start()

    def _now_ms(self):
        return int(time.time() * 1000)

    def _check_stale(self):
        if self._last_update_ms is None:
            return
        if (self._now_ms() - self._last_update_ms) > self._stale_timeout_ms:
            self.set_all_null()
            self._last_update_ms = None

    def set_all_null(self):
        for m in self.motors:
            m.set_null()

    def update_from_ros(self, speeds=None, angles_deg=None):
        """
        speeds: list/tuple/dict for indices 0..3
        angles_deg: list/tuple/dict for indices 0..3 (steering angle in degrees)
        """

        self._last_update_ms = self._now_ms()

        def _get(src, idx):
            if src is None:
                return None
            if isinstance(src, dict):
                return src.get(idx, None)
            if isinstance(src, (list, tuple)):
                return src[idx] if idx < len(src) else None
            return None

        for i in range(4):
            spd = _get(speeds, i)
            ang = _get(angles_deg, i)
            self.motors[i].set_values(spd, ang)
