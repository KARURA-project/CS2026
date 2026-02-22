# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'BatteryInfo.ui'
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
from PySide6.QtWidgets import (QApplication, QGridLayout, QSizePolicy, QWidget)

from batteryinfobox import BatteryInfoBox

class Ui_w_Motor_Battery_Info(object):
    def setupUi(self, w_Motor_Battery_Info):
        if not w_Motor_Battery_Info.objectName():
            w_Motor_Battery_Info.setObjectName(u"w_Motor_Battery_Info")
        w_Motor_Battery_Info.resize(400, 303)
        self.gridFrame = BatteryInfoBox(w_Motor_Battery_Info)
        self.gridFrame.setObjectName(u"gridFrame")
        self.gridFrame.setGeometry(QRect(20, 30, 291, 211))
        self.gridLayout = QGridLayout(self.gridFrame)
        self.gridLayout.setObjectName(u"gridLayout")

        self.retranslateUi(w_Motor_Battery_Info)

        QMetaObject.connectSlotsByName(w_Motor_Battery_Info)
    # setupUi

    def retranslateUi(self, w_Motor_Battery_Info):
        w_Motor_Battery_Info.setWindowTitle(QCoreApplication.translate("w_Motor_Battery_Info", u"Battery Information", None))
    # retranslateUi

