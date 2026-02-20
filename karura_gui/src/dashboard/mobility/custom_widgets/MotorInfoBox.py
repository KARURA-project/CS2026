# -*- coding: utf-8 -*-

from PySide6.QtCore import QCoreApplication, QMetaObject, QSize, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class Ui_MotorInfoBox(object):
    def setupUi(self, MotorInfoBox):
        if not MotorInfoBox.objectName():
            MotorInfoBox.setObjectName(u"MotorInfoBox")

        MotorInfoBox.resize(200, 120)
        MotorInfoBox.setMinimumSize(QSize(0, 120))
        MotorInfoBox.setMaximumSize(QSize(16777215, 180))
        MotorInfoBox.setAttribute(Qt.WA_StyledBackground, True)

        # Let karura_dark.qss drive styling (do not hardcode backgrounds here)
        MotorInfoBox.setStyleSheet(u"")

        # =========================
        # Layout
        # =========================
        self.verticalLayout = QVBoxLayout(MotorInfoBox)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout.setSpacing(8)

        # =========================
        # Title
        # =========================
        self.motor_name = QLabel(MotorInfoBox)
        self.motor_name.setObjectName(u"motor_name")
        f_title = QFont()
        f_title.setPointSize(11)
        f_title.setBold(True)
        self.motor_name.setFont(f_title)
        self.motor_name.setAlignment(Qt.AlignCenter)
        self.verticalLayout.addWidget(self.motor_name)

        # =========================
        # Speed row
        # =========================
        self.SpeedGroup = QGroupBox(MotorInfoBox)
        self.SpeedGroup.setObjectName(u"SpeedGroup")
        self.SpeedGroup.setTitle("")
        self.SpeedGroup.setFlat(True)
        self.SpeedGroup.setStyleSheet("QGroupBox{border:0;}")

        self.speedRow = QHBoxLayout(self.SpeedGroup)
        self.speedRow.setObjectName("speedRow")
        self.speedRow.setContentsMargins(0, 0, 0, 0)
        self.speedRow.setSpacing(6)

        self.speed_label = QLabel(self.SpeedGroup)
        self.speed_label.setObjectName(u"speed_label")
        self.speed_label.setText("Speed:")
        self.speedRow.addWidget(self.speed_label, 0, Qt.AlignLeft)

        self.speed_value = QLabel(self.SpeedGroup)
        self.speed_value.setObjectName(u"speed_value")
        self.speed_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        f_val = QFont()
        f_val.setPointSize(10)
        f_val.setBold(True)
        self.speed_value.setFont(f_val)
        self.speedRow.addWidget(self.speed_value, 1)

        self.verticalLayout.addWidget(self.SpeedGroup)

        # =========================
        # Angle row (steering angle)
        # =========================
        self.AngleGroup = QGroupBox(MotorInfoBox)
        self.AngleGroup.setObjectName(u"AngleGroup")
        self.AngleGroup.setTitle("")
        self.AngleGroup.setFlat(True)
        self.AngleGroup.setStyleSheet("QGroupBox{border:0;}")

        self.angleRow = QHBoxLayout(self.AngleGroup)
        self.angleRow.setObjectName("angleRow")
        self.angleRow.setContentsMargins(0, 0, 0, 0)
        self.angleRow.setSpacing(6)

        self.angle_label = QLabel(self.AngleGroup)
        self.angle_label.setObjectName(u"angle_label")
        self.angle_label.setText("Angle:")
        self.angleRow.addWidget(self.angle_label, 0, Qt.AlignLeft)

        self.angle_value = QLabel(self.AngleGroup)
        self.angle_value.setObjectName(u"angle_value")
        self.angle_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.angle_value.setFont(f_val)
        self.angleRow.addWidget(self.angle_value, 1)

        self.angle_units = QLabel(self.AngleGroup)
        self.angle_units.setObjectName(u"angle_units")
        self.angle_units.setText("deg")
        self.angle_units.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.angleRow.addWidget(self.angle_units, 0)

        self.verticalLayout.addWidget(self.AngleGroup)

        self.retranslateUi(MotorInfoBox)
        QMetaObject.connectSlotsByName(MotorInfoBox)

    def retranslateUi(self, MotorInfoBox):
        MotorInfoBox.setWindowTitle(QCoreApplication.translate("MotorInfoBox", u"MotorInfoBox", None))
        self.motor_name.setText(QCoreApplication.translate("MotorInfoBox", u"Motor", None))
        self.speed_value.setText(QCoreApplication.translate("MotorInfoBox", u"NULL", None))
        self.angle_value.setText(QCoreApplication.translate("MotorInfoBox", u"NULL", None))
