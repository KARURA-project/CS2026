# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'NetworkStatus.ui'
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
from PySide6.QtWidgets import (QApplication, QGroupBox, QHBoxLayout, QLabel,
    QSizePolicy, QSpacerItem, QVBoxLayout, QWidget)

from pathlib import Path

class Ui_NetworkStatus(object):
    def setupUi(self, NetworkStatus):
        if not NetworkStatus.objectName():
            NetworkStatus.setObjectName(u"NetworkStatus")
        NetworkStatus.resize(234, 93)
        NetworkStatus.setMinimumSize(QSize(0, 60))
        NetworkStatus.setMaximumSize(QSize(234, 93))
        NetworkStatus.setStyleSheet(u"background-color: rgb(54, 54, 54);")
        self.verticalLayout = QVBoxLayout(NetworkStatus)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.BatteryGroup = QGroupBox(NetworkStatus)
        self.BatteryGroup.setObjectName(u"BatteryGroup")
        self.BatteryGroup.setMinimumSize(QSize(0, 30))
        font = QFont()
        font.setPointSize(9)
        self.BatteryGroup.setFont(font)
        self.BatteryGroup.setStyleSheet(u"QGroupBox {\n"
"    border: 0px;\n"
"}")
        self.BatteryGroup.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.horizontalLayout = QHBoxLayout(self.BatteryGroup)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.battery_label = QLabel(self.BatteryGroup)
        self.battery_label.setObjectName(u"battery_label")
        self.battery_label.setMaximumSize(QSize(100, 16777215))
        palette = QPalette()
        brush = QBrush(QColor(255, 255, 255, 255))
        brush.setStyle(Qt.BrushStyle.SolidPattern)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, brush)
        brush1 = QBrush(QColor(54, 54, 54, 255))
        brush1.setStyle(Qt.BrushStyle.SolidPattern)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, brush1)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, brush)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, brush)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, brush1)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, brush1)
        brush2 = QBrush(QColor(255, 255, 255, 128))
        brush2.setStyle(Qt.BrushStyle.SolidPattern)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, brush)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, brush1)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, brush)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, brush)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, brush1)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, brush)
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, brush1)
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, brush)
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, brush)
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, brush1)
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        self.battery_label.setPalette(palette)
        self.battery_label.setStyleSheet(u"color: rgb(255, 255, 255);a")

        self.horizontalLayout.addWidget(self.battery_label)

        self.label = QLabel(self.BatteryGroup)
        self.label.setObjectName(u"label")
        self.label.setMaximumSize(QSize(20, 20))
        print(Path("qt_designer/icons/green_circle.png").exists())
        self.label.setPixmap(QPixmap("qt_designer/icons/green_circle.png"))
        print(self.label.pixmap)
        self.label.setScaledContents(True)

        self.horizontalLayout.addWidget(self.label)


        self.verticalLayout.addWidget(self.BatteryGroup)

        self.status_box = QGroupBox(NetworkStatus)
        self.status_box.setObjectName(u"status_box")
        self.status_box.setMinimumSize(QSize(0, 0))
        self.horizontalLayout_4 = QHBoxLayout(self.status_box)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(-1, 0, -1, 0)
        self.bandwidth_box = QGroupBox(self.status_box)
        self.bandwidth_box.setObjectName(u"bandwidth_box")
        self.bandwidth_box.setEnabled(True)
        self.bandwidth_box.setMinimumSize(QSize(90, 30))
        self.bandwidth_box.setMaximumSize(QSize(180, 30))
        self.bandwidth_box.setStyleSheet(u"QGroupBox {\n"
"    border: 0px;\n"
"}")
        self.bandwidth_box.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.bandwidth_box.setFlat(True)
        self.bandwidth_box.setCheckable(False)
        self.horizontalLayout_3 = QHBoxLayout(self.bandwidth_box)
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.bandwidth_val = QLabel(self.bandwidth_box)
        self.bandwidth_val.setObjectName(u"bandwidth_val")
        self.bandwidth_val.setMaximumSize(QSize(120, 16777215))
        palette1 = QPalette()
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, brush)
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, brush1)
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, brush)
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, brush)
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, brush1)
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, brush)
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, brush1)
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, brush)
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, brush)
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, brush1)
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        palette1.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, brush)
        palette1.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, brush1)
        palette1.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, brush)
        palette1.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, brush)
        palette1.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, brush1)
        palette1.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        self.bandwidth_val.setPalette(palette1)
        self.bandwidth_val.setStyleSheet(u"color: rgb(255, 255, 255);")
        self.bandwidth_val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_3.addWidget(self.bandwidth_val)


        self.horizontalLayout_4.addWidget(self.bandwidth_box)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer)

        self.latency_box = QGroupBox(self.status_box)
        self.latency_box.setObjectName(u"latency_box")
        self.latency_box.setEnabled(True)
        self.latency_box.setMinimumSize(QSize(90, 30))
        self.latency_box.setMaximumSize(QSize(120, 16777215))
        self.latency_box.setStyleSheet(u"QGroupBox {\n"
"    border: 0px;\n"
"}")
        self.latency_box.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.latency_box.setFlat(True)
        self.latency_box.setCheckable(False)
        self.horizontalLayout_2 = QHBoxLayout(self.latency_box)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.latency_val = QLabel(self.latency_box)
        self.latency_val.setObjectName(u"latency_val")
        self.latency_val.setMaximumSize(QSize(60, 16777215))
        palette2 = QPalette()
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, brush)
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, brush1)
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, brush)
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, brush)
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, brush1)
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, brush)
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, brush1)
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, brush)
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, brush)
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, brush1)
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        palette2.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, brush)
        palette2.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, brush1)
        palette2.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, brush)
        palette2.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, brush)
        palette2.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, brush1)
        palette2.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, brush1)
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette2.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.PlaceholderText, brush2)
#endif
        self.latency_val.setPalette(palette2)
        self.latency_val.setStyleSheet(u"color: rgb(255, 255, 255);")
        self.latency_val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_2.addWidget(self.latency_val)


        self.horizontalLayout_4.addWidget(self.latency_box)


        self.verticalLayout.addWidget(self.status_box)


        self.retranslateUi(NetworkStatus)

        QMetaObject.connectSlotsByName(NetworkStatus)
    # setupUi

    def retranslateUi(self, NetworkStatus):
        NetworkStatus.setWindowTitle(QCoreApplication.translate("NetworkStatus", u"Form", None))
        self.BatteryGroup.setTitle("")
        self.battery_label.setText(QCoreApplication.translate("NetworkStatus", u"Network Status", None))
        self.label.setText("")
        self.status_box.setTitle("")
        self.bandwidth_box.setTitle("")
        self.bandwidth_val.setText(QCoreApplication.translate("NetworkStatus", u"200/420 Mbps", None))
        self.latency_box.setTitle("")
        self.latency_val.setText(QCoreApplication.translate("NetworkStatus", u"(42ms)", None))
    # retranslateUi

