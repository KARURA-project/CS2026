# -*- coding: utf-8 -*-

################################################################################
## MobilityScreen.py (generated base + hand-edited layout)
################################################################################

from PySide6.QtCore import QCoreApplication, QMetaObject, QSize, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QSplitter, QVBoxLayout, QWidget
)

from dashboard.core import config
from .widgets import MotorInfoPanel, MainCameraPanel, IMUWidget, WASDWidget
from .custom_widgets.Camera import VideoWidget


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        # ============================================================
        # Main Window Setup
        # ============================================================
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")

        MainWindow.resize(1400, 900)
        MainWindow.setMinimumSize(QSize(1300, 900))

        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")

        self.rootLayout = QHBoxLayout(self.centralwidget)
        self.rootLayout.setContentsMargins(0, 0, 0, 0)
        self.rootLayout.setSpacing(0)

        # ============================================================
        # Splitter: LEFT telemetry | CENTER main view
        # ============================================================
        self.splitter = QSplitter(Qt.Orientation.Horizontal, self.centralwidget)
        self.splitter.setObjectName(u"mainSplitter")
        self.splitter.setChildrenCollapsible(False)
        self.rootLayout.addWidget(self.splitter)

        # ============================================================
        # LEFT PANEL: Telemetry
        # ============================================================
        self.leftgroup = QFrame(self.centralwidget)
        self.leftgroup.setObjectName(u"leftgroup")
        self.leftgroup.setFrameShape(QFrame.Shape.StyledPanel)

        # Important: constrain left size sanely
        self.leftgroup.setMinimumWidth(360)
        self.leftgroup.setMaximumWidth(520)

        self.leftLayout = QVBoxLayout(self.leftgroup)
        self.leftLayout.setObjectName(u"leftLayout")
        self.leftLayout.setContentsMargins(10, 10, 10, 10)
        self.leftLayout.setSpacing(12)
        self.leftLayout.setAlignment(Qt.AlignmentFlag.AlignTop)

        # IMU
        self.IMU = IMUWidget(self.leftgroup)
        self.IMU.setObjectName(u"IMUWidget")
        self.leftLayout.addWidget(self.IMU)

        # Battery/Motor panel
        self.BatteryData = MotorInfoPanel(self.leftgroup)
        self.BatteryData.setObjectName(u"BatteryData")

        # Demo items (remove later when data-driven)
        for _ in range(6):
            self.BatteryData.add_battery()

        self.leftLayout.addWidget(self.BatteryData)

        self.splitter.addWidget(self.leftgroup)

        # ============================================================
        # CENTER PANEL: Camera + HUD + Floating Bottom Bar
        # ============================================================
        self.centralgroup = QFrame(self.centralwidget)
        self.centralgroup.setObjectName(u"centralgroup")
        self.centralgroup.setFrameShape(QFrame.Shape.StyledPanel)

        self.centerLayout = QVBoxLayout(self.centralgroup)
        self.centerLayout.setObjectName(u"centerLayout")
        self.centerLayout.setContentsMargins(10, 10, 10, 10)
        self.centerLayout.setSpacing(12)

        # ------------------------------------------------------------
        # 1) MAIN CAMERA VIEW (must expand)
        # ------------------------------------------------------------
        self.maincameravideo = VideoWidget(source=config.RTSP_URL)
        self.maincameravideo.setObjectName(u"maincameravideo")

        # Force expansion (this is the key fix)
        self.maincameravideo.setMinimumSize(QSize(0, 0))
        self.maincameravideo.setMaximumSize(QSize(16777215, 16777215))
        self.maincameravideo.setSizePolicy(
            QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        )

        self.centerLayout.addWidget(self.maincameravideo, 1)  # stretch=1 makes it fill

        # ------------------------------------------------------------
        # 2) LOWER HUD DOCK (Switch Cameras + WASD)
        # ------------------------------------------------------------
        self.hudDock = QFrame(self.centralgroup)
        self.hudDock.setObjectName(u"hudDock")
        self.hudDock.setFixedHeight(210)

        self.hudDockLayout = QHBoxLayout(self.hudDock)
        self.hudDockLayout.setContentsMargins(10, 10, 10, 10)
        self.hudDockLayout.setSpacing(14)

        # Left: camera panel (contains Switch Cameras button)
        self.maincamerapanel = MainCameraPanel(self.hudDock)
        self.maincamerapanel.setObjectName(u"maincamerapanel")
        self.maincamerapanel.setFixedWidth(260)
        self.hudDockLayout.addWidget(self.maincamerapanel, 0, Qt.AlignLeft | Qt.AlignVCenter)

        self.hudDockLayout.addStretch(1)

        # Right: teleop visualizer
        self.wasdwidget = WASDWidget(self.hudDock)
        self.wasdwidget.setObjectName(u"wasdwidget")
        self.hudDockLayout.addWidget(self.wasdwidget, 0, Qt.AlignRight | Qt.AlignVCenter)

        self.centerLayout.addWidget(self.hudDock, 0)

        # ------------------------------------------------------------
        # 3) FLOATING BOTTOM BAR (Start | Timer Pill | Stop | LEDs)
        # ------------------------------------------------------------
        self.bottomBarRow = QHBoxLayout()
        self.bottomBarRow.setContentsMargins(0, 0, 0, 0)

        self.bottomBar = QFrame(self.centralgroup)
        self.bottomBar.setObjectName(u"bottomBar")
        self.bottomBar.setFixedHeight(86)
        self.bottomBar.setMaximumWidth(640)  # keep it floating + compact

        self.bottomBarLayout = QHBoxLayout(self.bottomBar)
        self.bottomBarLayout.setContentsMargins(18, 10, 18, 10)
        self.bottomBarLayout.setSpacing(10)

        # Start
        self.btnStart = QPushButton(self.bottomBar)
        self.btnStart.setObjectName(u"bottomStartButton")
        self.btnStart.setText("Start")
        self.btnStart.setFixedSize(QSize(90, 52))

        # Timer pill (time only)
        self.timerPill = QFrame(self.bottomBar)
        self.timerPill.setObjectName(u"missionTimerPill")
        self.timerPill.setFixedSize(QSize(170, 60))

        self.timerPillLayout = QVBoxLayout(self.timerPill)
        self.timerPillLayout.setContentsMargins(12, 6, 12, 6)
        self.timerPillLayout.setSpacing(0)

        self.timerValue = QLabel(self.timerPill)
        self.timerValue.setObjectName(u"missionTimerValue")
        self.timerValue.setText("0 / 6000s")
        self.timerValue.setAlignment(Qt.AlignCenter)

        f = QFont()
        f.setPointSize(15)
        f.setBold(True)
        self.timerValue.setFont(f)

        self.timerPillLayout.addWidget(self.timerValue)

        # Stop
        self.btnStop = QPushButton(self.bottomBar)
        self.btnStop.setObjectName(u"bottomStopButton")
        self.btnStop.setText("Stop")
        self.btnStop.setFixedSize(QSize(90, 52))

        # Add control cluster
        self.bottomBarLayout.addWidget(self.btnStart)
        self.bottomBarLayout.addWidget(self.timerPill)
        self.bottomBarLayout.addWidget(self.btnStop)

        # Spacer
        self.bottomBarLayout.addStretch(1)

        # Status cluster (LINK + RTSP)
        self.statusCluster = QWidget(self.bottomBar)
        self.statusCluster.setObjectName(u"statusCluster")
        self.statusClusterLayout = QHBoxLayout(self.statusCluster)
        self.statusClusterLayout.setContentsMargins(0, 0, 0, 0)
        self.statusClusterLayout.setSpacing(10)

        # LINK chip
        self.cmdStatusBox = QFrame(self.statusCluster)
        self.cmdStatusBox.setObjectName(u"statusBox")
        self.cmdStatusBoxLayout = QHBoxLayout(self.cmdStatusBox)
        self.cmdStatusBoxLayout.setContentsMargins(10, 6, 10, 6)
        self.cmdStatusBoxLayout.setSpacing(8)

        self.cmdLed = QLabel(self.cmdStatusBox)
        self.cmdLed.setObjectName(u"ledOk")
        self.cmdLed.setFixedSize(QSize(10, 10))

        self.cmdLabel = QLabel(self.cmdStatusBox)
        self.cmdLabel.setObjectName(u"statusLabel")
        self.cmdLabel.setText("LINK")

        self.cmdStatusBoxLayout.addWidget(self.cmdLed)
        self.cmdStatusBoxLayout.addWidget(self.cmdLabel)

        # RTSP chip
        self.rtspStatusBox = QFrame(self.statusCluster)
        self.rtspStatusBox.setObjectName(u"statusBox")
        self.rtspStatusBoxLayout = QHBoxLayout(self.rtspStatusBox)
        self.rtspStatusBoxLayout.setContentsMargins(10, 6, 10, 6)
        self.rtspStatusBoxLayout.setSpacing(8)

        self.rtspLed = QLabel(self.rtspStatusBox)
        self.rtspLed.setObjectName(u"ledBad")
        self.rtspLed.setFixedSize(QSize(10, 10))

        self.rtspLabel = QLabel(self.rtspStatusBox)
        self.rtspLabel.setObjectName(u"statusLabel")
        self.rtspLabel.setText("RTSP")

        self.rtspStatusBoxLayout.addWidget(self.rtspLed)
        self.rtspStatusBoxLayout.addWidget(self.rtspLabel)

        self.statusClusterLayout.addWidget(self.cmdStatusBox)
        self.statusClusterLayout.addWidget(self.rtspStatusBox)

        self.bottomBarLayout.addWidget(self.statusCluster)

        # Center the floating bar
        self.bottomBarRow.addStretch(1)
        self.bottomBarRow.addWidget(self.bottomBar, 0, Qt.AlignHCenter)
        self.bottomBarRow.addStretch(1)

        self.centerLayout.addLayout(self.bottomBarRow, 0)

        self.splitter.addWidget(self.centralgroup)

        # Give the camera panel most space by default
        self.splitter.setStretchFactor(0, 0)  # left fixed-ish
        self.splitter.setStretchFactor(1, 1)  # center grows
        self.splitter.setSizes([420, 1200])

        # ============================================================
        # Window chrome
        # ============================================================
        MainWindow.setCentralWidget(self.centralwidget)

        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        MainWindow.setMenuBar(self.menubar)

        self.retranslateUi(MainWindow)
        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Mobility Dashboard", None))
