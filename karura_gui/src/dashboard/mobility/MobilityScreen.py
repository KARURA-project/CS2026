# -*- coding: utf-8 -*-

from PySide6.QtCore import QCoreApplication, QMetaObject, QSize, Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QSplitter, QVBoxLayout, QWidget
)

from dashboard.core import config
from .widgets import MotorInfoPanel, DirectionWidget, BatteryStatusWidget, HelperBox, CameraToggleWidget
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
        # LEFT PANEL: Telemetry (Motors)
        # ============================================================
        self.leftgroup = QFrame(self.centralwidget)
        self.leftgroup.setObjectName(u"leftgroup")
        self.leftgroup.setFrameShape(QFrame.Shape.StyledPanel)

        self.leftgroup.setMinimumWidth(360)
        self.leftgroup.setMaximumWidth(520)

        self.leftLayout = QVBoxLayout(self.leftgroup)
        self.leftLayout.setObjectName(u"leftLayout")
        self.leftLayout.setContentsMargins(10, 10, 10, 10)
        self.leftLayout.setSpacing(12)
        self.leftLayout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.MotorPanel = MotorInfoPanel(self.leftgroup)
        self.MotorPanel.setObjectName(u"MotorPanel")
        self.leftLayout.addWidget(self.MotorPanel)

        self.HelperBox = HelperBox("Controls & Information", self.leftgroup)
        self.HelperBox.setObjectName(u"HelperBox")
        self.leftLayout.addWidget(self.HelperBox)

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
        # 1) MAIN CAMERA VIEW
        # ------------------------------------------------------------
        #Disabled RTSP temporarily and replaced with local camera
        #self.maincameravideo = VideoWidget(source=config.RTSP_URL)
        self.maincameravideo = VideoWidget(source=0)
        self.maincameravideo.setObjectName(u"maincameravideo")
        self.maincameravideo.setMinimumSize(QSize(0, 0))
        self.maincameravideo.setMaximumSize(QSize(16777215, 16777215))
        self.maincameravideo.setSizePolicy(
            QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        )
        self.centerLayout.addWidget(self.maincameravideo, 1)

        # ------------------------------------------------------------
        # 2) LOWER HUD DOCK (Battery | Controls | Direction)
        # ------------------------------------------------------------
        self.hudDock = QFrame(self.centralgroup)
        self.hudDock.setObjectName(u"hudDock")
        self.hudDock.setFixedHeight(210)

        self.hudDockLayout = QHBoxLayout(self.hudDock)
        self.hudDockLayout.setContentsMargins(10, 10, 10, 10)
        self.hudDockLayout.setSpacing(14)

        # Battery box LEFT
        self.batteryDock = QFrame(self.hudDock)
        self.batteryDock.setObjectName("batteryDock")
        self.batteryDock.setFixedSize(QSize(260, 170))
        batteryDockLayout = QVBoxLayout(self.batteryDock)
        batteryDockLayout.setContentsMargins(8, 8, 8, 8)
        batteryDockLayout.setSpacing(0)

        self.BatteryBox = BatteryStatusWidget(self.batteryDock)
        self.BatteryBox.setObjectName("BatteryBox")
        batteryDockLayout.addWidget(self.BatteryBox)

        self.hudDockLayout.addWidget(self.batteryDock, 0, Qt.AlignLeft | Qt.AlignVCenter)

        self.hudDockLayout.addStretch(1)

        #Container to allow vertical alignment of camera toggle button
        self.cameraToggleContainer = QWidget(self.hudDock)
        self.cameraToggleLayout = QVBoxLayout(self.cameraToggleContainer)
        self.cameraToggleLayout.setContentsMargins(0, 0, 0, 0)

        #Creates Camera Toggle Widget
        self.CameraToggleButton = CameraToggleWidget(self.cameraToggleContainer)
        self.CameraToggleButton.setObjectName("CameraToggleButton")
        self.CameraToggleButton.setFixedWidth(220) 

        self.cameraToggleLayout.addWidget(self.CameraToggleButton)

        self.hudDockLayout.addWidget(self.cameraToggleContainer, 0, Qt.AlignTop | Qt.AlignHCenter)

        self.hudDockLayout.addStretch(1)

        # Controls card MIDDLE
        # self.controlsDock = QFrame(self.hudDock)
        # self.controlsDock.setObjectName("controlsDock")
        # self.controlsDock.setFixedSize(QSize(360, 170))
        # controlsDockLayout = QVBoxLayout(self.controlsDock)
        # controlsDockLayout.setContentsMargins(8, 8, 8, 8)
        # controlsDockLayout.setSpacing(0)

        # self.ControlsHint = ControlsHintWidget(self.controlsDock)
        # self.ControlsHint.setObjectName("ControlsHint")
        # controlsDockLayout.addWidget(self.ControlsHint)

        # self.hudDockLayout.addStretch(1)
        # self.hudDockLayout.addWidget(self.controlsDock, 0, Qt.AlignVCenter)
        # self.hudDockLayout.addStretch(1)

        # Direction widget RIGHT
        self.DirectionWidget = DirectionWidget(self.hudDock)
        self.DirectionWidget.setObjectName(u"DirectionWidget")
        self.hudDockLayout.addWidget(self.DirectionWidget, 0, Qt.AlignRight | Qt.AlignVCenter)

        self.centerLayout.addWidget(self.hudDock, 0)


        # ------------------------------------------------------------
        # 3) FLOATING BOTTOM BAR (Switch | Start/Pause | Timer | Stop/Reset | LEDs)
        # ------------------------------------------------------------
        self.bottomBarRow = QHBoxLayout()
        self.bottomBarRow.setContentsMargins(0, 0, 0, 0)

        self.bottomBar = QFrame(self.centralgroup)
        self.bottomBar.setObjectName(u"bottomBar")
        self.bottomBar.setFixedHeight(86)
        self.bottomBar.setMaximumWidth(780)

        self.bottomBarLayout = QHBoxLayout(self.bottomBar)
        self.bottomBarLayout.setContentsMargins(18, 10, 18, 10)
        self.bottomBarLayout.setSpacing(10)

        self.btnSwitchCams = QPushButton(self.bottomBar)
        self.btnSwitchCams.setObjectName(u"bottomSwitchCamsButton")
        self.btnSwitchCams.setText("Switch Cameras")
        self.btnSwitchCams.setFixedSize(QSize(150, 52))

        self.btnStart = QPushButton(self.bottomBar)
        self.btnStart.setObjectName(u"bottomStartButton")
        self.btnStart.setText("Start")
        self.btnStart.setFixedSize(QSize(90, 52))

        self.timerPill = QFrame(self.bottomBar)
        self.timerPill.setObjectName(u"missionTimerPill")
        self.timerPill.setFixedSize(QSize(170, 60))

        self.timerPillLayout = QVBoxLayout(self.timerPill)
        self.timerPillLayout.setContentsMargins(12, 0, 12, 0)
        self.timerPillLayout.setSpacing(0)

        self.timerValue = QLabel(self.timerPill)
        self.timerValue.setObjectName(u"missionTimerValue")
        self.timerValue.setText("00:00.0")
        self.timerValue.setAlignment(Qt.AlignCenter)

        f = QFont()
        f.setPointSize(15)
        f.setBold(True)
        self.timerValue.setFont(f)

        self.timerPillLayout.addWidget(self.timerValue)

        self.btnStop = QPushButton(self.bottomBar)
        self.btnStop.setObjectName(u"bottomStopButton")
        self.btnStop.setText("Stop")
        self.btnStop.setFixedSize(QSize(90, 52))

        self.bottomBarLayout.addWidget(self.btnSwitchCams)
        self.bottomBarLayout.addWidget(self.btnStart)
        self.bottomBarLayout.addWidget(self.timerPill)
        self.bottomBarLayout.addWidget(self.btnStop)

        self.bottomBarLayout.addStretch(1)

        self.statusCluster = QWidget(self.bottomBar)
        self.statusCluster.setObjectName(u"statusCluster")
        self.statusClusterLayout = QHBoxLayout(self.statusCluster)
        self.statusClusterLayout.setContentsMargins(0, 0, 0, 0)
        self.statusClusterLayout.setSpacing(10)

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

        self.bottomBarRow.addStretch(1)
        self.bottomBarRow.addWidget(self.bottomBar, 0, Qt.AlignHCenter)
        self.bottomBarRow.addStretch(1)

        self.centerLayout.addLayout(self.bottomBarRow, 0)

        self.splitter.addWidget(self.centralgroup)

        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([420, 1200])

        # ============================================================
        # Window chrome
        # ============================================================
        MainWindow.setCentralWidget(self.centralwidget)

        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        MainWindow.setMenuBar(self.menubar)

        # ============================================================
        # Camera Power Toggle Wiring
        # ============================================================
        def on_camera_toggle(is_active):
            if is_active:
                self.maincameravideo.start_camera()
            else:
                self.maincameravideo.stop_camera()

        # Connect the toggle signal to our local handler
        self.CameraToggleButton.toggled.connect(on_camera_toggle)

        # Set the initial UI state to match the code's auto-start
        self.CameraToggleButton.set_state(True)

        # ============================================================
        # Camera switching wiring
        # ============================================================
        self._camera_sources = [config.RTSP_FRONT, config.RTSP_REAR, config.RTSP_ARM]
        self._camera_idx = 0

        def _cycle_camera():
            self._camera_idx = (self._camera_idx + 1) % len(self._camera_sources)
            new_url = self._camera_sources[self._camera_idx]
            self.maincameravideo.switch_camera(new_url)

        self.btnSwitchCams.clicked.connect(_cycle_camera)

        # ============================================================
        # Stopwatch
        # ============================================================
        self._sw_running = False
        self._sw_elapsed_ms = 0
        self._sw_tick_ms = 100

        self._sw_timer = QTimer(MainWindow)
        self._sw_timer.setInterval(self._sw_tick_ms)

        def _format_ms(ms: int) -> str:
            total = ms / 1000.0
            minutes = int(total // 60)
            seconds = total - (minutes * 60)
            return f"{minutes:02d}:{seconds:04.1f}"

        def _sw_tick():
            self._sw_elapsed_ms += self._sw_tick_ms
            self.timerValue.setText(_format_ms(self._sw_elapsed_ms))

        self._sw_timer.timeout.connect(_sw_tick)

        def _sw_toggle():
            if not self._sw_running:
                self._sw_timer.start()
                self._sw_running = True
                self.btnStart.setText("Pause")
            else:
                self._sw_timer.stop()
                self._sw_running = False
                self.btnStart.setText("Start")

        def _sw_reset():
            self._sw_timer.stop()
            self._sw_running = False
            self._sw_elapsed_ms = 0
            self.timerValue.setText("00:00.0")
            self.btnStart.setText("Start")

        self.btnStart.clicked.connect(_sw_toggle)
        self.btnStop.clicked.connect(_sw_reset)

        # Auto-start the default stream
        self.maincameravideo.start_camera()

        self.retranslateUi(MainWindow)
        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Mobility Dashboard", None))
