from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QWidget, QSizePolicy
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QGraphicsDropShadowEffect

class BottomBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("bottomBar")
        self.setFixedHeight(110)  # "real bar" feel

        # Floating shadow
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(24)
        shadow.setXOffset(0)
        shadow.setYOffset(-2)
        self.setGraphicsEffect(shadow)

        root = QHBoxLayout(self)
        root.setContentsMargins(18, 12, 18, 12)
        root.setSpacing(14)

        # ----- LEFT: Start -----
        self.btn_start = QPushButton("Start")
        self.btn_start.setObjectName("primaryButton")
        self.btn_start.setFixedSize(110, 60)

        # ----- CENTER: Mission Timer (bigger) + Stop -----
        center = QWidget()
        center.setObjectName("timerCluster")
        center_layout = QHBoxLayout(center)
        center_layout.setContentsMargins(14, 10, 14, 10)
        center_layout.setSpacing(12)

        self.timer_label = QLabel("Mission Timer")
        self.timer_label.setObjectName("muted")
        self.timer_label.setAlignment(Qt.AlignCenter)

        self.timer_value = QLabel("0 / 6000s")  # replace with your TimerBarWidget display
        self.timer_value.setObjectName("missionTimerValue")
        self.timer_value.setAlignment(Qt.AlignCenter)

        # Make timer text larger
        f = QFont()
        f.setPointSize(16)
        f.setBold(True)
        self.timer_value.setFont(f)

        timer_stack = QWidget()
        timer_stack_layout = QVBoxLayout(timer_stack)
        timer_stack_layout.setContentsMargins(0, 0, 0, 0)
        timer_stack_layout.setSpacing(2)
        timer_stack_layout.addWidget(self.timer_label)
        timer_stack_layout.addWidget(self.timer_value)

        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setObjectName("dangerButton")
        self.btn_stop.setFixedSize(110, 60)

        center_layout.addWidget(timer_stack, 1)
        center_layout.addWidget(self.btn_stop)

        center.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        # ----- RIGHT: Switch Cameras + status chips -----
        right = QWidget()
        right_layout = QHBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        self.btn_switch_cam = QPushButton("Switch Cameras")
        self.btn_switch_cam.setObjectName("switchCamButton")
        self.btn_switch_cam.setFixedSize(190, 60)

        # Optional status indicators (super useful on missions)
        self.ros_status = QLabel("ROS ●")
        self.ros_status.setObjectName("statusChipOk")
        self.rtsp_status = QLabel("RTSP ●")
        self.rtsp_status.setObjectName("statusChipWarn")  # flip to Ok when stream alive
        self.link_status = QLabel("LINK: ---")
        self.link_status.setObjectName("statusChip")

        right_layout.addWidget(self.btn_switch_cam)
        right_layout.addWidget(self.ros_status)
        right_layout.addWidget(self.rtsp_status)
        right_layout.addWidget(self.link_status)

        # ----- Assemble root -----
        root.addWidget(self.btn_start)
        root.addWidget(center, 1)
        root.addWidget(right)
