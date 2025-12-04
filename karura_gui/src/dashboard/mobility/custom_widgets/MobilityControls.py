# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MobilityControls.ui'
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
from PySide6.QtWidgets import (QApplication, QLabel, QSizePolicy, QWidget)

class Ui_MobilityControls(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"MobilityControls")
        Form.resize(436, 360)
        self.nountriangleup = QLabel(Form)
        self.nountriangleup.setObjectName(u"nountriangleup")
        self.nountriangleup.setGeometry(QRect(170, 50, 81, 71))
        self.nountriangleup.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-up.png"))
        self.nountriangleup.setScaledContents(True)
        self.nountriangledown = QLabel(Form)
        self.nountriangledown.setObjectName(u"nountriangledown")
        self.nountriangledown.setGeometry(QRect(170, 210, 81, 71))
        self.nountriangledown.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-down.png"))
        self.nountriangledown.setScaledContents(True)
        self.nountriangledown_2 = QLabel(Form)
        self.nountriangledown_2.setObjectName(u"nountriangledown_2")
        self.nountriangledown_2.setGeometry(QRect(80, 130, 81, 71))
        self.nountriangledown_2.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-left.png"))
        self.nountriangledown_2.setScaledContents(True)
        self.nountriangledown_3 = QLabel(Form)
        self.nountriangledown_3.setObjectName(u"nountriangledown_3")
        self.nountriangledown_3.setGeometry(QRect(260, 130, 81, 71))
        self.nountriangledown_3.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-right.png"))
        self.nountriangledown_3.setScaledContents(True)

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.nountriangleup.setText("")
        self.nountriangledown.setText("")
        self.nountriangledown_2.setText("")
        self.nountriangledown_3.setText("")
    # retranslateUi

