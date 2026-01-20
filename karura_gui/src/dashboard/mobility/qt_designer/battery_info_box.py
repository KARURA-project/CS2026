# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'BatteryInfoBox.ui'
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
from PySide6.QtWidgets import (QApplication, QFrame, QLabel, QProgressBar,
    QSizePolicy, QWidget)

class Ui_BatteryInfoBox(object):
    def setupUi(self, BatteryInfoBox):
        if not BatteryInfoBox.objectName():
            BatteryInfoBox.setObjectName(u"BatteryInfoBox")
        BatteryInfoBox.resize(200, 40)
        BatteryInfoBox.setMaximumSize(QSize(16777215, 40))
        BatteryInfoBox.setBaseSize(QSize(0, 0))
        BatteryInfoBox.setStyleSheet(u"background-color: rgb(54, 54, 54);")
        self.frame = QFrame(BatteryInfoBox)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(0, 0, 200, 100))
        self.frame.setAutoFillBackground(False)
        self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QFrame.Shadow.Raised)
        self.label = QLabel(self.frame)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 10, 51, 21))
        self.label.setStyleSheet(u"color: rgb(255, 255, 255);")
        self.progressBar = QProgressBar(self.frame)
        self.progressBar.setObjectName(u"progressBar")
        self.progressBar.setGeometry(QRect(70, 9, 121, 23))
        self.progressBar.setStyleSheet(u"QProgressBar {\n"
"    border: 2px solid #333;\n"
"    border-radius: 5px;\n"
"    background: #1a1a1a;\n"
"    text-align: center;\n"
"    color: white;\n"
"}\n"
"\n"
"/* GREEN (Healthy) */\n"
".battery-green QProgressBar::chunk {\n"
"    background-color: #00cc44;\n"
"}\n"
"\n"
"/* YELLOW (Warning) */\n"
".battery-yellow QProgressBar::chunk {\n"
"    background-color: #ffcc00;\n"
"}\n"
"\n"
"/* RED (Critical) */\n"
".battery-red QProgressBar::chunk {\n"
"    background-color: #ff3333;\n"
"}\n"
"")
        self.progressBar.setValue(24)

        self.retranslateUi(BatteryInfoBox)

        QMetaObject.connectSlotsByName(BatteryInfoBox)
    # setupUi

    def retranslateUi(self, BatteryInfoBox):
        BatteryInfoBox.setWindowTitle(QCoreApplication.translate("BatteryInfoBox", u"Form", None))
        self.label.setText(QCoreApplication.translate("BatteryInfoBox", u"Battery", None))
    # retranslateUi

