# -*- coding: utf-8 -*-
from PySide6.QtCore import (QCoreApplication, QMetaObject, QSize, Qt)
from PySide6.QtWidgets import (QGridLayout, QLabel, QWidget)

class Ui_WASDWidget(object):
    def setupUi(self, TeleopKeyWidget):
        if not TeleopKeyWidget.objectName():
            TeleopKeyWidget.setObjectName(u"TeleopKeyWidget")

        TeleopKeyWidget.resize(240, 220)
        TeleopKeyWidget.setMaximumWidth(280)

        self.gridLayout = QGridLayout(TeleopKeyWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setSpacing(8)

        # Base style (neutral key)
        self.base_style = """
            QLabel {
                background-color: #1E1617;
                color: #F2F2F2;
                border: 1px solid #3A2426;
                border-radius: 10px;
                font-weight: 800;
                font-size: 16px;
                padding: 6px;
            }
        """

        def mk_key(name: str):
            lbl = QLabel(TeleopKeyWidget)
            lbl.setObjectName(name)              # important for targeting
            lbl.setFixedSize(QSize(60, 60))
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(self.base_style)
            return lbl

        # 3x3 teleop grid: u i o / j k l / m , .
        self.keyU = mk_key("keyU")
        self.keyI = mk_key("keyI")
        self.keyO = mk_key("keyO")
        self.keyJ = mk_key("keyJ")
        self.keyK = mk_key("keyK")
        self.keyL = mk_key("keyL")
        self.keyM = mk_key("keyM")
        self.keyComma = mk_key("keyComma")
        self.keyDot = mk_key("keyDot")

        self.gridLayout.addWidget(self.keyU,     0, 0, 1, 1)
        self.gridLayout.addWidget(self.keyI,     0, 1, 1, 1)
        self.gridLayout.addWidget(self.keyO,     0, 2, 1, 1)
        self.gridLayout.addWidget(self.keyJ,     1, 0, 1, 1)
        self.gridLayout.addWidget(self.keyK,     1, 1, 1, 1)
        self.gridLayout.addWidget(self.keyL,     1, 2, 1, 1)
        self.gridLayout.addWidget(self.keyM,     2, 0, 1, 1)
        self.gridLayout.addWidget(self.keyComma, 2, 1, 1, 1)
        self.gridLayout.addWidget(self.keyDot,   2, 2, 1, 1)

        self.retranslateUi(TeleopKeyWidget)
        QMetaObject.connectSlotsByName(TeleopKeyWidget)

    def retranslateUi(self, TeleopKeyWidget):
        TeleopKeyWidget.setWindowTitle(QCoreApplication.translate("TeleopKeyWidget", u"Teleop Keys", None))
        self.keyU.setText(QCoreApplication.translate("TeleopKeyWidget", u"U", None))
        self.keyI.setText(QCoreApplication.translate("TeleopKeyWidget", u"I", None))
        self.keyO.setText(QCoreApplication.translate("TeleopKeyWidget", u"O", None))
        self.keyJ.setText(QCoreApplication.translate("TeleopKeyWidget", u"J", None))
        self.keyK.setText(QCoreApplication.translate("TeleopKeyWidget", u"K", None))
        self.keyL.setText(QCoreApplication.translate("TeleopKeyWidget", u"L", None))
        self.keyM.setText(QCoreApplication.translate("TeleopKeyWidget", u"M", None))
        self.keyComma.setText(QCoreApplication.translate("TeleopKeyWidget", u",", None))
        self.keyDot.setText(QCoreApplication.translate("TeleopKeyWidget", u".", None))
