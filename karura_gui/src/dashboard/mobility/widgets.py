from PySide6.QtWidgets import QWidget, QLabel, QGridLayout, QSizePolicy, QVBoxLayout
import PySide6.QtCore
from custom_widgets.MotorInfoBox import Ui_MotorInfoBox
from custom_widgets.NetworkStatus import Ui_NetworkStatus
from custom_widgets.CameraSwitchButton import Ui_CameraSwitchButton
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
    def __init__(self):
        super().__init__()

        self.boxes = []  
        self.columns = 2

        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.setFixedSize(400, 400)
        self.setMaximumHeight(1000)

    def update_ui(self):
        #Manually updates the size of the box
        self.setFixedSize(400, 100 * math.ceil(len(self.boxes) / self.columns))

    def add_battery(self):
        box = MotorInfoBox()

        #Makes background color show
        box.setAttribute(PySide6.QtCore.Qt.WA_StyledBackground, True)

        # FORCE every widget to expand but NEVER overlap
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        index = len(self.boxes)
        self.boxes.append(box)

        row = index // self.columns
        col = index % self.columns

        self.update_ui()

        self.layout.addWidget(box, row, col)

        return box
