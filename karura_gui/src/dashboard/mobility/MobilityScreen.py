# # -*- coding: utf-8 -*-

# ################################################################################
# ## Form generated from reading UI file 'MobilityScreen.ui'
# ##
# ## Created by: Qt User Interface Compiler version 6.10.0
# ##
# ## WARNING! All changes made in this file will be lost when recompiling UI file!
# ################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGroupBox, QHBoxLayout,
    QMainWindow, QMenuBar, QSizePolicy, QStatusBar,
    QVBoxLayout, QWidget, QSplitter)

from .widgets import MotorInfoPanel, MainCameraPanel, MobilityControls, IMUWidget, TimerButtonPanel, WASDWidget, TimerBarWidget
from .custom_widgets.Camera import VideoWidget
from .custom_widgets.Terminal import PrimitiveTerminalWidget, DEFAULT_TTY_CMD


# class Ui_MainWindow(object):
#     def setupUi(self, MainWindow):
#         if not MainWindow.objectName():
#             MainWindow.setObjectName(u"MainWindow")
#         MainWindow.resize(1024, 768)
#         MainWindow.setMinimumSize(QSize(900, 600))
#         self.centralwidget = QWidget(MainWindow)
#         self.centralwidget.setObjectName(u"centralwidget")
#         self.horizontalLayout = QHBoxLayout(self.centralwidget)
#         self.horizontalLayout.setObjectName(u"horizontalLayout")

#         # Left Panel (Telemetry)
#         self.leftgroup = QFrame(self.centralwidget)
#         self.leftgroup.setObjectName(u"leftgroup")
#         self.leftgroup.setMinimumSize(QSize(300, 550))
#         self.leftgroup.setMaximumSize(QSize(500, 16777215))
#         self.leftgroup.setFrameShape(QFrame.Shape.StyledPanel)
#         self.leftgroup.setFrameShadow(QFrame.Shadow.Raised)
#         self.verticalLayout = QVBoxLayout(self.leftgroup)
#         self.verticalLayout.setObjectName(u"verticalLayout")
#         self.verticalLayout.setAlignment(Qt.AlignmentFlag.AlignCenter)

#         # Secondary Camera Panel (on the left)
#         self.SecondaryCam = VideoWidget(None, 1)
#         self.SecondaryCam.setObjectName(u"frame_3")
#         self.SecondaryCam.setMaximumSize(QSize(360, 360))
#         self.verticalLayout.addWidget(self.SecondaryCam)

#         # IMU Data Panel
#         self.IMU = IMUWidget(self.leftgroup)
#         self.IMU.setObjectName(u"IMUWidget")
#         self.IMU.setMinimumSize(QSize(0, 400))
#         self.verticalLayout.addWidget(self.IMU)

#         # Battery Data Panel
#         self.BatteryData = MotorInfoPanel(self.leftgroup)
#         self.BatteryData.setObjectName(u"BatteryData")
#         self.BatteryData.setMinimumSize(QSize(200, 200))
#         self.BatteryData.add_battery()
#         self.BatteryData.add_battery()
#         self.BatteryData.add_battery()
#         self.BatteryData.add_battery()
#         self.BatteryData.add_battery()
#         self.BatteryData.add_battery()
#         self.verticalLayout.addWidget(self.BatteryData)

#         self.horizontalLayout.addWidget(self.leftgroup)

#         # Central Panel (Camera Views & Controls)
#         self.centralgroup = QFrame(self.centralwidget)
#         self.centralgroup.setObjectName(u"centralgroup")
#         self.centralgroup.setMinimumSize(QSize(400, 550))
#         self.centralgroup.setSizeIncrement(QSize(100, 0))
#         self.centralgroup.setFrameShape(QFrame.Shape.StyledPanel)
#         self.centralgroup.setFrameShadow(QFrame.Shadow.Raised)
#         self.verticalLayout_4 = QVBoxLayout(self.centralgroup)
#         self.verticalLayout_4.setObjectName(u"verticalLayout_4")

#         # Top Row: Main Camera + Secondary Camera (even width, centered)
#         self.cameraLayout = QHBoxLayout()
#         self.maincameravideo = VideoWidget(None, 0)
#         self.maincameravideo.setObjectName(u"maincameravideo")
#         self.maincameravideo.setMaximumSize(QSize(640, 360))

#         # Adjust the secondary camera to match the size and position next to main camera
#         self.SecondaryCam.setMaximumSize(QSize(640, 360))

#         self.cameraLayout.addWidget(self.maincameravideo)
#         self.cameraLayout.addWidget(self.SecondaryCam)
#         self.cameraLayout.setSpacing(10)
#         self.cameraLayout.setAlignment(Qt.AlignmentFlag.AlignCenter)

#         self.verticalLayout_4.addLayout(self.cameraLayout)

#         # Main Camera Panel (below camera views)
#         self.maincamerapanel = MainCameraPanel(self.centralgroup)
#         self.maincamerapanel.setObjectName(u"maincamerapanel")
#         self.maincamerapanel.setMaximumSize(QSize(300, 150))
#         self.verticalLayout_4.addWidget(self.maincamerapanel)

#         self.horizontalLayout.addWidget(self.centralgroup)

#         # Right Panel (Aerial Info & Terminal)
#         self.rightgroup = QFrame(self.centralwidget)
#         self.rightgroup.setObjectName(u"rightgroup")
#         sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
#         sizePolicy1.setHorizontalStretch(0)
#         sizePolicy1.setVerticalStretch(0)
#         sizePolicy1.setHeightForWidth(self.rightgroup.sizePolicy().hasHeightForWidth())
#         self.rightgroup.setSizePolicy(sizePolicy1)
#         self.rightgroup.setMinimumSize(QSize(300, 550))
#         self.rightgroup.setMaximumSize(QSize(400, 16777215))
#         self.rightgroup.setLayoutDirection(Qt.LayoutDirection.LeftToRight)
#         self.rightgroup.setStyleSheet(u"align-items: center;")
#         self.rightgroup.setFrameShape(QFrame.Shape.StyledPanel)
#         self.rightgroup.setFrameShadow(QFrame.Shadow.Raised)
#         self.rightgroupLayout = QVBoxLayout(self.rightgroup)
#         self.verticalLayout_2 = QVBoxLayout(self.rightgroup)
#         self.verticalLayout_2.setObjectName(u"verticalLayout_2")
#         self.verticalLayout_2.setContentsMargins(0, -1, 0, -1)
#         self.groupBox = QGroupBox(self.rightgroup)
#         self.groupBox.setObjectName(u"groupBox")
#         self.groupBox.setMinimumSize(QSize(300, 360))
#         self.groupBox.setMaximumSize(QSize(300, 360))
#         self.verticalLayout_3 = QVBoxLayout(self.groupBox)
#         self.verticalLayout_3.setSpacing(0)
#         self.verticalLayout_3.setObjectName(u"verticalLayout_3")
#         self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)

#         # Aerial Image & Info
#         self.AerialImg = QFrame(self.groupBox)
#         self.AerialImg.setObjectName(u"AerialImg")
#         self.AerialImg.setMinimumSize(QSize(0, 300))
#         self.AerialImg.setMaximumSize(QSize(300, 300))
#         self.AerialImg.setStyleSheet(u"align-items: center;")
#         self.AerialImg.setFrameShape(QFrame.Shape.StyledPanel)
#         self.AerialImg.setFrameShadow(QFrame.Shadow.Raised)

#         self.verticalLayout_3.addWidget(self.AerialImg)

#         self.AerialInfo = QFrame(self.groupBox)
#         self.AerialInfo.setObjectName(u"AerialInfo")
#         self.AerialInfo.setMinimumSize(QSize(0, 40))
#         self.AerialInfo.setMaximumSize(QSize(16777215, 40))
#         self.AerialInfo.setFrameShape(QFrame.Shape.StyledPanel)
#         self.AerialInfo.setFrameShadow(QFrame.Shadow.Raised)

#         self.verticalLayout_3.addWidget(self.AerialInfo)

#         self.verticalLayout_2.addWidget(self.groupBox)
#         self.horizontalLayout.addWidget(self.rightgroup)

#         MainWindow.setCentralWidget(self.centralwidget)
#         self.menubar = QMenuBar(MainWindow)
#         self.menubar.setObjectName(u"menubar")
#         self.menubar.setGeometry(QRect(0, 0, 1024, 19))
#         MainWindow.setMenuBar(self.menubar)
#         self.statusbar = QStatusBar(MainWindow)
#         self.statusbar.setObjectName(u"statusbar")
#         MainWindow.setStatusBar(self.statusbar)

#         self.retranslateUi(MainWindow)

#         QMetaObject.connectSlotsByName(MainWindow)

#     def retranslateUi(self, MainWindow):
#         MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
#         self.groupBox.setTitle("")
from dashboard.core import config


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1024, 768)
        MainWindow.setMinimumSize(QSize(1300, 1000))

        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")

        # --- Replace the main HBox layout with a horizontal splitter ---
        self.rootLayout = QHBoxLayout(self.centralwidget)
        self.rootLayout.setObjectName(u"rootLayout")

        self.splitter = QSplitter(Qt.Orientation.Horizontal, self.centralwidget)
        self.splitter.setObjectName(u"mainSplitter")
        self.rootLayout.addWidget(self.splitter)

        # =========================
        # Left Panel (Telemetry)
        # =========================
        self.leftgroup = QFrame(self.centralwidget)
        self.leftgroup.setObjectName(u"leftgroup")
        self.leftgroup.setMinimumSize(QSize(250, 550))
        self.leftgroup.setMaximumSize(QSize(600, 16777215))  # optional cap
        self.leftgroup.setFrameShape(QFrame.Shape.StyledPanel)
        self.leftgroup.setFrameShadow(QFrame.Shadow.Raised)

        self.verticalLayout = QVBoxLayout(self.leftgroup)
        self.verticalLayout.setObjectName(u"verticalLayout")

        # Align the layout's content to the horizontal center
        self.verticalLayout.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        # self.verticalLayout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.verticalLayout.setAlignment(Qt.AlignmentFlag.AlignTop)

        # REMOVE SecondaryCam entirely (do not create it)

        # IMU Data Panel
        self.IMU = IMUWidget(self.leftgroup)
        self.IMU.setObjectName(u"IMUWidget")
        self.IMU.setMinimumSize(QSize(0, 400))
        self.verticalLayout.addWidget(self.IMU)

        # Battery Data Panel
        self.BatteryData = MotorInfoPanel(self.leftgroup)
        self.BatteryData.setObjectName(u"BatteryData")
        self.BatteryData.setMinimumSize(QSize(200, 200))
        self.BatteryData.add_battery()
        self.BatteryData.add_battery()
        self.BatteryData.add_battery()
        self.BatteryData.add_battery()
        self.BatteryData.add_battery()
        self.BatteryData.add_battery()
        self.verticalLayout.addWidget(self.BatteryData)

        # Add left panel to splitter
        self.splitter.addWidget(self.leftgroup)

        # =========================
        # Central Panel (Camera + Controls)
        # =========================
        self.centralgroup = QFrame(self.centralwidget)
        self.centralgroup.setMinimumSize(QSize(400, 550))
        self.centralgroup.setFrameShape(QFrame.Shape.StyledPanel)

        # Main Vertical Layout for the whole frame
        self.verticalLayout_Main = QVBoxLayout(self.centralgroup)

        # 1. Add the BIG camera view to the top
        self.maincameravideo = VideoWidget(source=config.RTSP_URL)
        camPolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.maincameravideo.setSizePolicy(camPolicy)
        self.verticalLayout_Main.addWidget(self.maincameravideo, 1) # Stretch = 1

        # 2. Create the Horizontal Layout for the bottom section
        self.bottomHorizontalLayout = QHBoxLayout() # No parent in constructor

        # 3. Create the Vertical Layout for the Panels (Camera Panel + Timer)
        self.panelStackLayout = QVBoxLayout() # No parent in constructor

        self.panelStackLayout.addStretch(1)

        self.maincamerapanel = MainCameraPanel(self.centralgroup)
        #self.maincamerapanel.setFixedHeight(180)
        self.maincamerapanel.setMaximumSize(QSize(300, 100))
        self.panelStackLayout.addWidget(self.maincamerapanel)

        self.timerbuttonpanel = TimerButtonPanel(self.centralgroup)
        self.timerbuttonpanel.setMaximumSize(QSize(300, 120))
        self.panelStackLayout.addWidget(self.timerbuttonpanel)

        self.timerprogressbar = TimerBarWidget(6000, self.centralgroup)

        # 4. Create the WASD Widget
        self.wasdwidget = WASDWidget(self.centralgroup)

        # 5. Add and set up horizontal layout
        self.bottomHorizontalLayout.addLayout(self.panelStackLayout, 1)
        self.bottomHorizontalLayout.addStretch(1)
        self.bottomHorizontalLayout.addWidget(self.wasdwidget, 0)

        # Second, add that entire Horizontal row to the Main Vertical layout
        self.verticalLayout_Main.addLayout(self.bottomHorizontalLayout, 0)

        # Add timerprogressbar to the bottom
        self.verticalLayout_Main.addWidget(self.timerprogressbar)

        # Add central panel to splitter
        self.splitter.addWidget(self.centralgroup)
        self.splitter.setSizes([320, 704])

        # --- Right panel removed بالكامل (do not create rightgroup) ---

        MainWindow.setCentralWidget(self.centralwidget)

        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1024, 19))
        MainWindow.setMenuBar(self.menubar)

        # self.statusbar = QStatusBar(MainWindow)
        # self.statusbar.setObjectName(u"statusbar")
        # MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
