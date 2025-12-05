################################################################################
## Form generated from reading UI file 'MobilityControls.ui'
##
## Created by: Qt User Interface Compiler version 6.10.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt, Signal)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform, QMouseEvent)
from PySide6.QtWidgets import (QApplication, QSizePolicy, QWidget, QLabel)

class ClickableQLabel(QLabel):
    # Define a signal that will be emitted when the label is clicked
    # It passes no data, but you could add arguments (e.g., self) if needed.
    clicked = Signal() 

    def __init__(self, parent=None):
        super().__init__(parent)
        # Optional: Change the cursor to indicate it's clickable
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mousePressEvent(self, event: QMouseEvent):
        """
        Re-implements the mouse press event handler.
        This method is called automatically when the mouse is clicked over the widget.
        """
        # We only care about the left button press
        if event.button() == Qt.MouseButton.LeftButton:
            # Emit the custom signal
            self.clicked.emit()
            # Call the base class method to ensure normal event propagation
            super().mousePressEvent(event)

class Ui_MobilityControls(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"MobilityControls")
        Form.resize(436, 360)
        
        # Up Button
        self.triangleup = ClickableQLabel(Form)
        self.triangleup.setObjectName(u"triangleup")
        self.triangleup.setGeometry(QRect(170, 50, 81, 71))
        self.triangleup.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-up.png"))
        self.triangleup.setScaledContents(True)
        
        # Down Button
        self.triangledown = ClickableQLabel(Form)
        self.triangledown.setObjectName(u"triangledown")
        self.triangledown.setGeometry(QRect(170, 210, 81, 71))
        self.triangledown.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-down.png"))
        self.triangledown.setScaledContents(True)
        
        # Left Button
        self.triangleleft = ClickableQLabel(Form)
        self.triangleleft.setObjectName(u"triangleleft")
        self.triangleleft.setGeometry(QRect(80, 130, 81, 71))
        self.triangleleft.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-left.png"))
        self.triangleleft.setScaledContents(True)
        
        # Right Button
        self.triangleright = ClickableQLabel(Form)
        self.triangleright.setObjectName(u"triangleright")
        self.triangleright.setGeometry(QRect(260, 130, 81, 71))
        self.triangleright.setPixmap(QPixmap(u"qt_designer/icons/noun-triangle-right.png"))
        self.triangleright.setScaledContents(True)

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    
    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.triangleup.setText("")
        self.triangledown.setText("")
        self.triangleleft.setText("")
        self.triangleright.setText("")