from PySide6.QtWidgets import QWidget, QLabel, QGridLayout, QSizePolicy, QVBoxLayout
from PySide6.QtCore import Signal, Slot, Qt, QCoreApplication
from .custom_widgets.MotorInfoBox import Ui_MotorInfoBox
from .custom_widgets.CameraSwitchButton import Ui_CameraSwitchButton
from .custom_widgets.MobilityControls import Ui_MobilityControls
from .custom_widgets.IMUWidget import Ui_IMUWidget
from .custom_widgets.TimerButtonPanel import Ui_TimerButtonPanel
from .custom_widgets.WASDWidget import Ui_WASDWidget
import math

#Initalizes QT widgets
class MotorInfoBox(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_MotorInfoBox()
        self.ui.setupUi(self)

# class NetworkStatus(QWidget):
#     def __init__(self, parent=None):
#         super().__init__(parent)

#         #Sets the UI to use the one made by designer
#         self.ui = Ui_NetworkStatus()
#         self.ui.setupUi(self)


class CameraSwitchButton(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_CameraSwitchButton()
        self.ui.setupUi(self)

class TimerButtonPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_TimerButtonPanel()
        self.ui.setupUi(self)

class WASDWidget(QWidget):
    # Signal: sends key string (e.g. "W") and state (True for Green, False for Gray)
    keyStateChanged = Signal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)

        # Sets the UI to use the one made by designer
        self.ui = Ui_WASDWidget()
        self.ui.setupUi(self)

        # Dictionary to map strings to the UI objects
        self.key_map = {
            "W": self.ui.labelW,
            "A": self.ui.labelA,
            "S": self.ui.labelS,
            "D": self.ui.labelD
        }

        # Connect signal to the slot that changes color
        self.keyStateChanged.connect(self.set_key_active)

    @Slot(str, bool)
    def set_key_active(self, key, active):
        key = key.upper()
        if key in self.key_map:
            label = self.key_map[key]
            if active:
                # Flip to Green
                label.setStyleSheet(self.ui.base_style + "QLabel { background-color: #4CAF50; color: white; border-color: #388E3C; }")
            else:
                # Flip back to Gray
                label.setStyleSheet(self.ui.base_style)

    # Example: Override keyboard events to test the signal
    def keyPressEvent(self, event):
        char = event.text().upper()
        self.keyStateChanged.emit(char, True)

    def keyReleaseEvent(self, event):
        char = event.text().upper()
        self.keyStateChanged.emit(char, False)

class MobilityControls(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_MobilityControls()
        self.ui.setupUi(self)

        self.ui.triangledown.clicked.connect(lambda: print("Down clicked"))
        self.ui.triangleup.clicked.connect(lambda: print("Up clicked"))
        self.ui.triangleleft.clicked.connect(lambda: print("Left clicked"))
        self.ui.triangleright.clicked.connect(lambda: print("Right clicked"))

class IMUWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_IMUWidget()
        self.ui.setupUi(self)

#Initalizes python widgets
class CameraWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.label = QLabel("Camera")

#Initalizes panels
class MainCameraPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout()
        layout.setSpacing(0)          # optional: no spacing
        layout.setContentsMargins(0,0,0,0)  # optional: flush edges

        self.camera_switch_button = CameraSwitchButton()
        # self.network_status = NetworkStatus()

        layout.addWidget(self.camera_switch_button)
        # layout.addWidget(self.network_status)
        
        self.setLayout(layout)
        self.setMaximumWidth(200)
        self.setMaximumHeight(170)

class MotorInfoPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.motors = []  
        self.columns = 2

        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.setFixedSize(400, 400)
        self.setMaximumHeight(1000)

    def update_ui(self):
        #Manually updates the size of the box
        self.setFixedSize(400, 100 * math.ceil(len(self.motors) / self.columns))

    def add_battery(self):
        mot = MotorInfoBox()

        #Makes background color show
        mot.setAttribute(Qt.WA_StyledBackground, True)

        # FORCE every widget to expand but NEVER overlap
        mot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        index = len(self.motors)
        self.motors.append(mot)

        row = index // self.columns
        col = index % self.columns

        self.update_ui()

        self.layout.addWidget(mot, row, col)

        return mot
    
    def update_values(self, arr):
        """
        Update each motor's speed and battery display.

        Only updates if `arr` length matches `self.motors`. Each entry in `arr` must be an
        object with `.speed` and `.battery` values.
        """

        #Checks if the arr length equals the number of batteries
        if(len(arr) != len(self.motors)):
            return
        
        #Updates each value. 
        for i in range(0, len(arr)):
            print(arr[i])
            self.motors[i].ui.speed_value.setText(QCoreApplication.translate("MotorInfoBox", u"{value}".format(value = arr[i]["speed"]), None))
            self.motors[i].ui.battery_bar.setValue(arr[i]["battery"])
        
