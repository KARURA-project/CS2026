# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'IMUWidget.ui'
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
from PySide6.QtWidgets import (QApplication, QGridLayout, QGroupBox, QLabel,
    QSizePolicy, QWidget, QVBoxLayout)

class Ui_IMUWidget(object):
    def setupUi(self, Ui_IMUWidget):
        # 1. Basic Window Setup
        if not Ui_IMUWidget.objectName():
            Ui_IMUWidget.setObjectName(u"Ui_IMUWidget")
        Ui_IMUWidget.resize(460, 500)  # Increased width slightly to fit your diagram minimum
        Ui_IMUWidget.setMaximumHeight(550)

        # 2. Define the Group Boxes
        self.IMUDiagram = QGroupBox(Ui_IMUWidget)
        self.IMUDiagram.setMinimumWidth(450)
        self.IMUDiagram.setMinimumHeight(350)

        self.IMUGeneralInfo = QGroupBox(Ui_IMUWidget)
        self.IMUGeneralInfo.setMaximumHeight(50)

        # 3. Setup the Grid Layout inside General Info
        self.IMUGeneralInfogridLayout = QGridLayout(self.IMUGeneralInfo)


        # 4. Setup Labels (Roll, Pitch, Yaw)
        self.IMURoverRoll = QLabel(self.IMUGeneralInfo)
        self.IMURoverRoll.setObjectName(u"IMURoverRoll")
        self.IMURoverRoll.setText("Roll: 0.0")  # Added text so it's not invisible

        self.IMURoverPitch = QLabel(self.IMUGeneralInfo)
        self.IMURoverPitch.setObjectName(u"IMURoverPitch")
        self.IMURoverPitch.setText("Pitch: 0.0") # Added text

        self.IMURoverYaw = QLabel(self.IMUGeneralInfo)
        self.IMURoverYaw.setObjectName(u"IMURoverYaw")
        self.IMURoverYaw.setText("Yaw: 0.0")   # Added text

        # 5. Add Labels to the General Info Grid
        self.IMUGeneralInfogridLayout.addWidget(self.IMURoverRoll, 0, 0)
        self.IMUGeneralInfogridLayout.addWidget(self.IMURoverPitch, 0, 1)
        self.IMUGeneralInfogridLayout.addWidget(self.IMURoverYaw, 0, 2)

        # 6. MASTER LAYOUT SETUP (This was the missing part)
        self.IMUgridLayout = QVBoxLayout(Ui_IMUWidget)
        
        # Add the two main sections to the master layout
        self.IMUgridLayout.addWidget(self.IMUDiagram)
        self.IMUgridLayout.addWidget(self.IMUGeneralInfo)

        self.mars_rover_image = QLabel(self.IMUDiagram)
        self.mars_rover_image.setObjectName(u"mars_rover_image")
        self.mars_rover_image.setGeometry(QRect(110, 90, 211, 171))
        self.mars_rover_image.setPixmap(QPixmap(u"qt_designer/icons/rover.png"))
        self.mars_rover_image.setScaledContents(True)
        self.FrontLeftWheelBox = QGroupBox(self.IMUDiagram)
        self.FrontLeftWheelBox.setObjectName(u"FrontLeftWheelBox")
        self.FrontLeftWheelBox.setGeometry(QRect(0, 10, 161, 81))
        self.gridLayout = QGridLayout(self.FrontLeftWheelBox)
        self.gridLayout.setObjectName(u"gridLayout")
        self.FrontLeftWheelLabel = QLabel(self.FrontLeftWheelBox)
        self.FrontLeftWheelLabel.setObjectName(u"FrontLeftWheelLabel")

        self.gridLayout.addWidget(self.FrontLeftWheelLabel, 0, 0, 1, 2)

        self.FrontLeftWheelActualLabel = QLabel(self.FrontLeftWheelBox)
        self.FrontLeftWheelActualLabel.setObjectName(u"FrontLeftWheelActualLabel")

        self.gridLayout.addWidget(self.FrontLeftWheelActualLabel, 1, 0, 1, 1)

        self.FrontLeftWheelActualValue = QLabel(self.FrontLeftWheelBox)
        self.FrontLeftWheelActualValue.setObjectName(u"FrontLeftWheelActualValue")

        self.gridLayout.addWidget(self.FrontLeftWheelActualValue, 1, 1, 1, 1)

        self.FrontLeftWheelTargetLabel = QLabel(self.FrontLeftWheelBox)
        self.FrontLeftWheelTargetLabel.setObjectName(u"FrontLeftWheelTargetLabel")

        self.gridLayout.addWidget(self.FrontLeftWheelTargetLabel, 2, 0, 1, 1)

        self.FrontLeftWheelTargetValue = QLabel(self.FrontLeftWheelBox)
        self.FrontLeftWheelTargetValue.setObjectName(u"FrontLeftWheelTargetValue")

        self.gridLayout.addWidget(self.FrontLeftWheelTargetValue, 2, 1, 1, 1)

        self.FrontRightWheelBox = QGroupBox(self.IMUDiagram)
        self.FrontRightWheelBox.setObjectName(u"FrontRightWheelBox")
        self.FrontRightWheelBox.setGeometry(QRect(280, 10, 161, 81))
        self.gridLayout_3 = QGridLayout(self.FrontRightWheelBox)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.FrontRightWheelLabel = QLabel(self.FrontRightWheelBox)
        self.FrontRightWheelLabel.setObjectName(u"FrontRightWheelLabel")

        self.gridLayout_3.addWidget(self.FrontRightWheelLabel, 0, 0, 1, 2)

        self.FrontRightWheelActualLabel = QLabel(self.FrontRightWheelBox)
        self.FrontRightWheelActualLabel.setObjectName(u"FrontRightWheelActualLabel")

        self.gridLayout_3.addWidget(self.FrontRightWheelActualLabel, 1, 0, 1, 1)

        self.FrontRightWheelActualValue = QLabel(self.FrontRightWheelBox)
        self.FrontRightWheelActualValue.setObjectName(u"FrontRightWheelActualValue")

        self.gridLayout_3.addWidget(self.FrontRightWheelActualValue, 1, 1, 1, 1)

        self.FrontRightWheelTargetLabel = QLabel(self.FrontRightWheelBox)
        self.FrontRightWheelTargetLabel.setObjectName(u"FrontRightWheelTargetLabel")

        self.gridLayout_3.addWidget(self.FrontRightWheelTargetLabel, 2, 0, 1, 1)

        self.FrontRightWheelTargetValue = QLabel(self.FrontRightWheelBox)
        self.FrontRightWheelTargetValue.setObjectName(u"FrontRightWheelTargetValue")

        self.gridLayout_3.addWidget(self.FrontRightWheelTargetValue, 2, 1, 1, 1)

        self.BackLeftWheelBox = QGroupBox(self.IMUDiagram)
        self.BackLeftWheelBox.setObjectName(u"BackLeftWheelBox")
        self.BackLeftWheelBox.setGeometry(QRect(10, 260, 161, 81))
        self.gridLayout_4 = QGridLayout(self.BackLeftWheelBox)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.BackLeftWheelLabel = QLabel(self.BackLeftWheelBox)
        self.BackLeftWheelLabel.setObjectName(u"BackLeftWheelLabel")

        self.gridLayout_4.addWidget(self.BackLeftWheelLabel, 0, 0, 1, 2)

        self.BackLeftWheelActualLabel = QLabel(self.BackLeftWheelBox)
        self.BackLeftWheelActualLabel.setObjectName(u"BackLeftWheelActualLabel")

        self.gridLayout_4.addWidget(self.BackLeftWheelActualLabel, 1, 0, 1, 1)

        self.BackLeftWheelActualValue = QLabel(self.BackLeftWheelBox)
        self.BackLeftWheelActualValue.setObjectName(u"BackLeftWheelActualValue")

        self.gridLayout_4.addWidget(self.BackLeftWheelActualValue, 1, 1, 1, 1)

        self.BackLeftWheelActualTarget = QLabel(self.BackLeftWheelBox)
        self.BackLeftWheelActualTarget.setObjectName(u"BackLeftWheelActualTarget")

        self.gridLayout_4.addWidget(self.BackLeftWheelActualTarget, 2, 0, 1, 1)

        self.BackLeftWheelTargetValue = QLabel(self.BackLeftWheelBox)
        self.BackLeftWheelTargetValue.setObjectName(u"BackLeftWheelTargetValue")

        self.gridLayout_4.addWidget(self.BackLeftWheelTargetValue, 2, 1, 1, 1)

        self.BackRightWheelBox = QGroupBox(self.IMUDiagram)
        self.BackRightWheelBox.setObjectName(u"BackRightWheelBox")
        self.BackRightWheelBox.setGeometry(QRect(270, 260, 161, 81))
        self.gridLayout_6 = QGridLayout(self.BackRightWheelBox)
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.BackRightWheelLabel = QLabel(self.BackRightWheelBox)
        self.BackRightWheelLabel.setObjectName(u"BackRightWheelLabel")

        self.gridLayout_6.addWidget(self.BackRightWheelLabel, 0, 0, 1, 2)

        self.BackRightWheelActualLabel = QLabel(self.BackRightWheelBox)
        self.BackRightWheelActualLabel.setObjectName(u"BackRightWheelActualLabel")

        self.gridLayout_6.addWidget(self.BackRightWheelActualLabel, 1, 0, 1, 1)

        self.BackRightWheelActualValue = QLabel(self.BackRightWheelBox)
        self.BackRightWheelActualValue.setObjectName(u"BackRightWheelActualValue")

        self.gridLayout_6.addWidget(self.BackRightWheelActualValue, 1, 1, 1, 1)

        self.BackRightWheelTargetLabel = QLabel(self.BackRightWheelBox)
        self.BackRightWheelTargetLabel.setObjectName(u"BackRightWheelTargetLabel")

        self.gridLayout_6.addWidget(self.BackRightWheelTargetLabel, 2, 0, 1, 1)

        self.BackRightWheelTargetValue = QLabel(self.BackRightWheelBox)
        self.BackRightWheelTargetValue.setObjectName(u"BackRightWheelTargetValue")

        self.gridLayout_6.addWidget(self.BackRightWheelTargetValue, 2, 1, 1, 1)


        self.retranslateUi(Ui_IMUWidget)

        QMetaObject.connectSlotsByName(Ui_IMUWidget)
    # setupUi

    def retranslateUi(self, Ui_IMUWidget):
        Ui_IMUWidget.setWindowTitle(QCoreApplication.translate("Ui_IMUWidget", u"Form", None))
        self.mars_rover_image.setText("")
        self.FrontLeftWheelBox.setTitle("")
        self.FrontLeftWheelLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Front Left Wheel", None))
        self.FrontLeftWheelActualLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Actual:", None))
        self.FrontLeftWheelActualValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
        self.FrontLeftWheelTargetLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Target:", None))
        self.FrontLeftWheelTargetValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
        self.FrontRightWheelBox.setTitle("")
        self.FrontRightWheelLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Front Right Wheel", None))
        self.FrontRightWheelActualLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Actual:", None))
        self.FrontRightWheelActualValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
        self.FrontRightWheelTargetLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Target:", None))
        self.FrontRightWheelTargetValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
        self.BackLeftWheelBox.setTitle("")
        self.BackLeftWheelLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Back Left Wheel", None))
        self.BackLeftWheelActualLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Actual:", None))
        self.BackLeftWheelActualValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
        self.BackLeftWheelActualTarget.setText(QCoreApplication.translate("Ui_IMUWidget", u"Target:", None))
        self.BackLeftWheelTargetValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
        self.BackRightWheelBox.setTitle("")
        self.BackRightWheelLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Back Right Wheel", None))
        self.BackRightWheelActualLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Actual:", None))
        self.BackRightWheelActualValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
        self.BackRightWheelTargetLabel.setText(QCoreApplication.translate("Ui_IMUWidget", u"Target:", None))
        self.BackRightWheelTargetValue.setText(QCoreApplication.translate("Ui_IMUWidget", u"TextLabel", None))
    # retranslateUi

