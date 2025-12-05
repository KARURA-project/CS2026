from PySide6.QtWidgets import QWidget, QLabel, QGridLayout, QSizePolicy, QVBoxLayout
import PySide6.QtCore
from custom_widgets.MotorInfoBox import Ui_MotorInfoBox
from custom_widgets.NetworkStatus import Ui_NetworkStatus
from custom_widgets.CameraSwitchButton import Ui_CameraSwitchButton
from custom_widgets.MobilityControls import Ui_MobilityControls
from custom_widgets.IMUWidget import Ui_IMUWidget
import math

#Initalizes QT widgets
class MotorInfoBox(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_MotorInfoBox()
        self.ui.setupUi(self)

class NetworkStatus(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_NetworkStatus()
        self.ui.setupUi(self)


class CameraSwitchButton(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #Sets the UI to use the one made by designer
        self.ui = Ui_CameraSwitchButton()
        self.ui.setupUi(self)

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
        self.network_status = NetworkStatus()

        layout.addWidget(self.camera_switch_button)
        layout.addWidget(self.network_status)
        
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
        mot.setAttribute(PySide6.QtCore.Qt.WA_StyledBackground, True)

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
            self.motors[i].ui.speed_value.setText(PySide6.QtCore.QCoreApplication.translate("MotorInfoBox", u"{value}".format(value = arr[i]["speed"]), None))
            self.motors[i].ui.battery_bar.setValue(arr[i]["battery"])
        
