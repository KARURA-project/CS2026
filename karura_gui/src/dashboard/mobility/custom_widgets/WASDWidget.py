# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'WASDWidget.ui'
################################################################################

from PySide6.QtCore import (QCoreApplication, QMetaObject, QSize, Qt)
from PySide6.QtWidgets import (QGridLayout, QLabel, QWidget)

class Ui_WASDWidget(object):
    def setupUi(self, WASDWidget):
        if not WASDWidget.objectName():
            WASDWidget.setObjectName(u"WASDWidget")
        
        # Enforcing the size constraints
        WASDWidget.resize(220, 150)
        WASDWidget.setMaximumWidth(250)
        
        self.gridLayout = QGridLayout(WASDWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setSpacing(10)

        # Common style for the keys
        self.base_style = """
            QLabel {
                background-color: #E0E0E0;
                color: #333333;
                border: 2px solid #BDBDBD;
                border-radius: 8px;
                font-weight: bold;
                font-size: 18px;
            }
        """

        # Creating the 4 labels
        self.labelW = QLabel(WASDWidget)
        self.labelW.setObjectName(u"labelW")
        self.labelW.setFixedSize(QSize(60, 60))
        self.labelW.setAlignment(Qt.AlignCenter)
        self.labelW.setStyleSheet(self.base_style)
        self.gridLayout.addWidget(self.labelW, 0, 1, 1, 1)

        self.labelA = QLabel(WASDWidget)
        self.labelA.setObjectName(u"labelA")
        self.labelA.setFixedSize(QSize(60, 60))
        self.labelA.setAlignment(Qt.AlignCenter)
        self.labelA.setStyleSheet(self.base_style)
        self.gridLayout.addWidget(self.labelA, 1, 0, 1, 1)

        self.labelS = QLabel(WASDWidget)
        self.labelS.setObjectName(u"labelS")
        self.labelS.setFixedSize(QSize(60, 60))
        self.labelS.setAlignment(Qt.AlignCenter)
        self.labelS.setStyleSheet(self.base_style)
        self.gridLayout.addWidget(self.labelS, 1, 1, 1, 1)

        self.labelD = QLabel(WASDWidget)
        self.labelD.setObjectName(u"labelD")
        self.labelD.setFixedSize(QSize(60, 60))
        self.labelD.setAlignment(Qt.AlignCenter)
        self.labelD.setStyleSheet(self.base_style)
        self.gridLayout.addWidget(self.labelD, 1, 2, 1, 1)

        self.retranslateUi(WASDWidget)
        QMetaObject.connectSlotsByName(WASDWidget)
    # setupUi

    def retranslateUi(self, WASDWidget):
        WASDWidget.setWindowTitle(QCoreApplication.translate("WASDWidget", u"WASD Indicator", None))
        self.labelW.setText(QCoreApplication.translate("WASDWidget", u"W", None))
        self.labelA.setText(QCoreApplication.translate("WASDWidget", u"A", None))
        self.labelS.setText(QCoreApplication.translate("WASDWidget", u"S", None))
        self.labelD.setText(QCoreApplication.translate("WASDWidget", u"D", None))
    # retranslateUi