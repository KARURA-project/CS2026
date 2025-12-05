# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MobilityScreen.ui'
##
## Created by: Qt User Interface Compiler version 6.10.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGroupBox, QHBoxLayout,
    QMainWindow, QMenuBar, QSizePolicy, QStatusBar,
    QVBoxLayout, QWidget)

from widgets import MotorInfoPanel, MainCameraPanel, MobilityControls, IMUWidget
from custom_widgets.Camera import VideoWidget
from custom_widgets.Terminal import PrimitiveTerminalWidget, DEFAULT_TTY_CMD

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1024, 768)
        MainWindow.setMinimumSize(QSize(900, 600))
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.leftgroup = QFrame(self.centralwidget)
        self.leftgroup.setObjectName(u"leftgroup")
        self.leftgroup.setMinimumSize(QSize(300, 550))
        self.leftgroup.setMaximumSize(QSize(500, 16777215))
        self.leftgroup.setFrameShape(QFrame.Shape.StyledPanel)
        self.leftgroup.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout = QVBoxLayout(self.leftgroup)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.SecondaryCam = QFrame(self.leftgroup)
        self.SecondaryCam.setObjectName(u"SecondaryCam")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.SecondaryCam.sizePolicy().hasHeightForWidth())
        # self.SecondaryCam.setSizePolicy(sizePolicy)
        # self.SecondaryCam.setMinimumSize(QSize(100, 100))
        # self.SecondaryCam.setMaximumSize(QSize(150, 150))
        # self.SecondaryCam.setFrameShape(QFrame.Shape.StyledPanel)
        # self.SecondaryCam.setFrameShadow(QFrame.Shadow.Raised)

        self.SecondaryCam = VideoWidget(None, 1)
        self.SecondaryCam.setObjectName(u"frame_3")
        self.SecondaryCam.setMaximumSize(QSize(360,360)) 


        self.verticalLayout.addWidget(self.SecondaryCam)

        # self.IMU = QFrame(self.leftgroup)
        # self.IMU.setObjectName(u"IMU")
        # self.IMU.setMinimumSize(QSize(0, 400))
        # self.IMU.setFrameShape(QFrame.Shape.StyledPanel)
        # self.IMU.setFrameShadow(QFrame.Shadow.Raised)

        self.IMU = IMUWidget(self.leftgroup)
        self.IMU.setObjectName(u"IMUWidget")
        self.IMU.setMinimumSize(QSize(0, 400))

        self.verticalLayout.addWidget(self.IMU)

        # self.BatteryData = QFrame(self.leftgroup)
        # self.BatteryData.setObjectName(u"BatteryData")
        # self.BatteryData.setMinimumSize(QSize(0, 200))
        # self.BatteryData.setFrameShape(QFrame.Shape.StyledPanel)
        # self.BatteryData.setFrameShadow(QFrame.Shadow.Raised)
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


        self.horizontalLayout.addWidget(self.leftgroup)

        self.centralgroup = QFrame(self.centralwidget)
        self.centralgroup.setObjectName(u"centralgroup")
        self.centralgroup.setMinimumSize(QSize(400, 550))
        self.centralgroup.setSizeIncrement(QSize(100, 0))
        self.centralgroup.setFrameShape(QFrame.Shape.StyledPanel)
        self.centralgroup.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_4 = QVBoxLayout(self.centralgroup)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        # self.frame_2 = QFrame(self.centralgroup)
        # self.frame_2.setObjectName(u"frame_2")
        # self.frame_2.setMaximumSize(QSize(400, 300))
        # self.frame_2.setFrameShape(QFrame.Shape.StyledPanel)
        # self.frame_2.setFrameShadow(QFrame.Shadow.Raised)

        self.frame2 = MainCameraPanel(self.centralgroup)
        self.frame2.setObjectName(u"frame_2")
        self.frame2.setMaximumSize(QSize(300,150))

        self.verticalLayout_4.addWidget(self.frame2)

        # self.frame = QFrame(self.centralgroup)
        # self.frame.setObjectName(u"frame")
        # self.frame.setMaximumSize(QSize(300, 300))
        # self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        # self.frame.setFrameShadow(QFrame.Shadow.Raised)

        self.frame = MobilityControls(self.centralgroup)
        self.frame.setObjectName(u"frame_2")
        self.frame.setMaximumSize(QSize(400,300))

        self.verticalLayout_4.addWidget(self.frame)

        self.frame3 = VideoWidget(None, 0)
        self.frame3.setObjectName(u"frame_3")
        self.frame.setMaximumSize(QSize(640,360)) 

        self.verticalLayout_4.addWidget(self.frame3)

        self.horizontalLayout.addWidget(self.centralgroup)

        self.rightgroup = QFrame(self.centralwidget)
        self.rightgroup.setObjectName(u"rightgroup")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.rightgroup.sizePolicy().hasHeightForWidth())
        self.rightgroup.setSizePolicy(sizePolicy1)
        self.rightgroup.setMinimumSize(QSize(300, 550))
        self.rightgroup.setMaximumSize(QSize(400, 16777215))
        self.rightgroup.setLayoutDirection(Qt.LayoutDirection.LeftToRight)
        self.rightgroup.setStyleSheet(u"align-items: center;")
        self.rightgroup.setFrameShape(QFrame.Shape.StyledPanel)
        self.rightgroup.setFrameShadow(QFrame.Shadow.Raised)
        self.rightgroupLayout = QVBoxLayout(self.rightgroup)
        self.verticalLayout_2 = QVBoxLayout(self.rightgroup)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, -1, 0, -1)
        self.groupBox = QGroupBox(self.rightgroup)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setMinimumSize(QSize(300, 360))
        self.groupBox.setMaximumSize(QSize(300, 360))
        self.verticalLayout_3 = QVBoxLayout(self.groupBox)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.AerialImg = QFrame(self.groupBox)
        self.AerialImg.setObjectName(u"AerialImg")
        self.AerialImg.setMinimumSize(QSize(0, 300))
        self.AerialImg.setMaximumSize(QSize(300, 300))
        self.AerialImg.setStyleSheet(u"align-items: center;")
        self.AerialImg.setFrameShape(QFrame.Shape.StyledPanel)
        self.AerialImg.setFrameShadow(QFrame.Shadow.Raised)

        self.verticalLayout_3.addWidget(self.AerialImg)

        self.AerialInfo = QFrame(self.groupBox)
        self.AerialInfo.setObjectName(u"AerialInfo")
        self.AerialInfo.setMinimumSize(QSize(0, 40))
        self.AerialInfo.setMaximumSize(QSize(16777215, 40))
        self.AerialInfo.setFrameShape(QFrame.Shape.StyledPanel)
        self.AerialInfo.setFrameShadow(QFrame.Shadow.Raised)

        self.verticalLayout_3.addWidget(self.AerialInfo)

        self.Terminal = PrimitiveTerminalWidget()
        self.Terminal.spawn(DEFAULT_TTY_CMD) #Spawns terminal
        self.verticalLayout_3.addWidget(self.Terminal)


        self.verticalLayout_2.addWidget(self.groupBox)


        self.horizontalLayout.addWidget(self.rightgroup)
    

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1024, 19))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.groupBox.setTitle("")
    # retranslateUi

