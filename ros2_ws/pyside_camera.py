import sys
import cv2
import rclpy

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtMultimedia import QCamera, QMediaDevices, QMediaCaptureSession
from PySide6.QtMultimediaWidgets import QVideoWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera GUI")
        



app = QApplication(sys.argv)

window = MainWindow()


window.show()

app.exec()