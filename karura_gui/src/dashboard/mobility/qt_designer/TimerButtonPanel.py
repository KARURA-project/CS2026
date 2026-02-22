# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'TimerButton.ui'
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
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QPushButton, QSizePolicy,
    QSpacerItem, QWidget)

class Ui_TimerButtonPanel(object):
    def setupUi(self, TimerButtonPanel):
        if not TimerButtonPanel.objectName():
            TimerButtonPanel.setObjectName(u"TimerButtonPanel")
        TimerButtonPanel.resize(202, 142)
        self.horizontalLayout_2 = QHBoxLayout(TimerButtonPanel)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer)

        self.pushButton_2 = QPushButton(TimerButtonPanel)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setMinimumSize(QSize(80, 80))
        self.pushButton_2.setMaximumSize(QSize(80, 80))

        self.horizontalLayout_2.addWidget(self.pushButton_2)

        self.horizontalSpacer_3 = QSpacerItem(20, 20, QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_3)

        self.pushButton = QPushButton(TimerButtonPanel)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setMinimumSize(QSize(80, 80))
        self.pushButton.setMaximumSize(QSize(80, 80))

        self.horizontalLayout_2.addWidget(self.pushButton)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_2)


        self.retranslateUi(TimerButtonPanel)

        QMetaObject.connectSlotsByName(TimerButtonPanel)
    # setupUi

    def retranslateUi(self, TimerButtonPanel):
        TimerButtonPanel.setWindowTitle(QCoreApplication.translate("TimerButtonPanel", u"TimerButtonPanel", None))
        self.pushButton_2.setText(QCoreApplication.translate("TimerButtonPanel", u"Start", None))
        self.pushButton.setText(QCoreApplication.translate("TimerButtonPanel", u"Stop", None))
    # retranslateUi

