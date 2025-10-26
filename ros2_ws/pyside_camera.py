import sys
import cv2

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtMultimedia import QCamera, QMediaDevices, QMediaCaptureSession
from PySide6.QtMultimediaWidgets import QVideoWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Camera")
        camera_device = QMediaDevices.defaultVideoInput()
        if camera_device.isNull():
            print("Error: No default camera found")

        self.camera = QCamera(camera_device)

        self.cap = QMediaCaptureSession()
        self.cap.setCamera(self.camera)

        vid = QVideoWidget()
        self.cap.setVideoOutput(vid)

        self.setCentralWidget(vid)

        self.camera.start()

app = QApplication(sys.argv)

window = MainWindow()


window.show()

app.exec()