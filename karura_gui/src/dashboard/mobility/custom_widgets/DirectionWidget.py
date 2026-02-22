# -*- coding: utf-8 -*-

from PySide6.QtCore import QCoreApplication, QMetaObject, QSize, Qt
from PySide6.QtWidgets import QGridLayout, QLabel, QWidget


class Ui_DirectionWidget(object):
    def setupUi(self, DirectionWidget):
        if not DirectionWidget.objectName():
            DirectionWidget.setObjectName(u"DirectionWidget")

        DirectionWidget.resize(340, 260)
        DirectionWidget.setMaximumWidth(380)

        self.gridLayout = QGridLayout(DirectionWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setSpacing(10)
        self.gridLayout.setContentsMargins(6, 6, 6, 6)

        # Keep columns consistent so center arrows line up
        for c in range(5):
            self.gridLayout.setColumnStretch(c, 1)
        self.gridLayout.setColumnMinimumWidth(2, 18)  # visual spacer in the middle

        # Base style (neutral key)
        self.base_style = """
            QLabel {
                background-color: #1E1617;
                color: #F2F2F2;
                border: 1px solid #3A2426;
                border-radius: 10px;
                font-weight: 800;
                font-size: 18px;
                padding: 6px;
            }
        """

        def mk_key(name: str):
            lbl = QLabel(DirectionWidget)
            lbl.setObjectName(name)
            lbl.setFixedSize(QSize(64, 64))
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(self.base_style)
            return lbl

        # Row 0
        self.keySpeedDownTop = mk_key("keySpeedDownTop")
        self.keyForward      = mk_key("keyForward")
        self.keySpeedUpTop   = mk_key("keySpeedUpTop")

        # Row 1
        self.keyRotateLeft   = mk_key("keyRotateLeft")
        self.keyLeft         = mk_key("keyLeft")
        self.keyRight        = mk_key("keyRight")
        self.keyRotateRight  = mk_key("keyRotateRight")

        # Row 2
        self.keySpeedDownBot = mk_key("keySpeedDownBot")
        self.keyBack         = mk_key("keyBack")
        self.keySpeedUpBot   = mk_key("keySpeedUpBot")

        # --- Placement (5 cols: 0 1 2 3 4; col 2 is spacer) ---
        # KEEP arrows + rotate exactly the same.
        # ONLY change +/- positions inward.

        # Top row:
        # Before: [-] at col0, [▲] at col2, [+] at col4
        # After:  [-] at col1, [▲] at col2, [+] at col3
        self.gridLayout.addWidget(self.keySpeedDownTop, 0, 1, 1, 1)
        self.gridLayout.addWidget(self.keyForward,      0, 2, 1, 1)
        self.gridLayout.addWidget(self.keySpeedUpTop,   0, 3, 1, 1)

        # Middle row stays identical:
        # [⟲] at col0, [◀] at col1, spacer col2, [▶] at col3, [⟳] at col4
        self.gridLayout.addWidget(self.keyRotateLeft,   1, 0, 1, 1)
        self.gridLayout.addWidget(self.keyLeft,         1, 1, 1, 1)
        self.gridLayout.addWidget(self.keyRight,        1, 3, 1, 1)
        self.gridLayout.addWidget(self.keyRotateRight,  1, 4, 1, 1)

        # Bottom row:
        # Before: [-] at col0, [▼] at col2, [+] at col4
        # After:  [-] at col1, [▼] at col2, [+] at col3
        self.gridLayout.addWidget(self.keySpeedDownBot, 2, 1, 1, 1)
        self.gridLayout.addWidget(self.keyBack,         2, 2, 1, 1)
        self.gridLayout.addWidget(self.keySpeedUpBot,   2, 3, 1, 1)

        # Spacer cell in the center row / middle column (unchanged)
        self.spacer = QLabel(DirectionWidget)
        self.spacer.setFixedSize(QSize(18, 18))
        self.spacer.setStyleSheet("background: transparent; border: 0;")
        self.gridLayout.addWidget(self.spacer, 1, 2, 1, 1)

        self.retranslateUi(DirectionWidget)
        QMetaObject.connectSlotsByName(DirectionWidget)

    def retranslateUi(self, DirectionWidget):
        DirectionWidget.setWindowTitle(QCoreApplication.translate("DirectionWidget", u"Direction", None))

        self.keySpeedDownTop.setText(QCoreApplication.translate("DirectionWidget", u"−", None))
        self.keySpeedUpTop.setText(QCoreApplication.translate("DirectionWidget", u"+", None))
        self.keySpeedDownBot.setText(QCoreApplication.translate("DirectionWidget", u"−", None))
        self.keySpeedUpBot.setText(QCoreApplication.translate("DirectionWidget", u"+", None))

        self.keyForward.setText(QCoreApplication.translate("DirectionWidget", u"▲", None))
        self.keyBack.setText(QCoreApplication.translate("DirectionWidget", u"▼", None))
        self.keyLeft.setText(QCoreApplication.translate("DirectionWidget", u"◀", None))
        self.keyRight.setText(QCoreApplication.translate("DirectionWidget", u"▶", None))

        self.keyRotateLeft.setText(QCoreApplication.translate("DirectionWidget", u"⟲", None))
        self.keyRotateRight.setText(QCoreApplication.translate("DirectionWidget", u"⟳", None))
