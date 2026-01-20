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
    QSizePolicy, QWidget)
import mobility_icons_rc
import mobility_icons_rc

class Ui_IMUData(object):
    def setupUi(self, IMUData):
        if not IMUData.objectName():
            IMUData.setObjectName(u"IMUData")
        IMUData.resize(443, 348)
        self.mars_rover_image = QLabel(IMUData)
        self.mars_rover_image.setObjectName(u"mars_rover_image")
        self.mars_rover_image.setGeometry(QRect(110, 90, 211, 171))
        self.mars_rover_image.setPixmap(QPixmap(u":/rover/rover.png"))
        self.mars_rover_image.setScaledContents(True)
        self.FrontLeftWheelBox = QGroupBox(IMUData)
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

        self.FrontRightWheelBox = QGroupBox(IMUData)
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

        self.BackLeftWheelBox = QGroupBox(IMUData)
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

        self.BackRightWheelBox = QGroupBox(IMUData)
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


        self.retranslateUi(IMUData)

        QMetaObject.connectSlotsByName(IMUData)
    # setupUi

    def retranslateUi(self, IMUData):
        IMUData.setWindowTitle(QCoreApplication.translate("IMUData", u"Form", None))
        self.mars_rover_image.setText("")
        self.FrontLeftWheelBox.setTitle("")
        self.FrontLeftWheelLabel.setText(QCoreApplication.translate("IMUData", u"Front Left Wheel", None))
        self.FrontLeftWheelActualLabel.setText(QCoreApplication.translate("IMUData", u"Actual:", None))
        self.FrontLeftWheelActualValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
        self.FrontLeftWheelTargetLabel.setText(QCoreApplication.translate("IMUData", u"Target:", None))
        self.FrontLeftWheelTargetValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
        self.FrontRightWheelBox.setTitle("")
        self.FrontRightWheelLabel.setText(QCoreApplication.translate("IMUData", u"Front Right Wheel", None))
        self.FrontRightWheelActualLabel.setText(QCoreApplication.translate("IMUData", u"Actual:", None))
        self.FrontRightWheelActualValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
        self.FrontRightWheelTargetLabel.setText(QCoreApplication.translate("IMUData", u"Target:", None))
        self.FrontRightWheelTargetValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
        self.BackLeftWheelBox.setTitle("")
        self.BackLeftWheelLabel.setText(QCoreApplication.translate("IMUData", u"Back Left Wheel", None))
        self.BackLeftWheelActualLabel.setText(QCoreApplication.translate("IMUData", u"Actual:", None))
        self.BackLeftWheelActualValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
        self.BackLeftWheelActualTarget.setText(QCoreApplication.translate("IMUData", u"Target:", None))
        self.BackLeftWheelTargetValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
        self.BackRightWheelBox.setTitle("")
        self.BackRightWheelLabel.setText(QCoreApplication.translate("IMUData", u"Back Right Wheel", None))
        self.BackRightWheelActualLabel.setText(QCoreApplication.translate("IMUData", u"Actual:", None))
        self.BackRightWheelActualValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
        self.BackRightWheelTargetLabel.setText(QCoreApplication.translate("IMUData", u"Target:", None))
        self.BackRightWheelTargetValue.setText(QCoreApplication.translate("IMUData", u"TextLabel", None))
    # retranslateUi

