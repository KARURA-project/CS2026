import sys
import cv2
import numpy as np
from PySide6.QtCore import (
    QThread,
    Signal,
    Slot,
    QWaitCondition,
    QMutex,
    Qt
)
from PySide6.QtGui import (
    QImage,
    QPixmap
)
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QVBoxLayout,
    QWidget,
    QMessageBox
)

# --- 1. Worker Thread for Video Capture ---

class CameraWorker(QThread):
    """
    Main worker thread for CameraWorker for video capture. Takes in backend source (rtsp server) 
    in order to display video footage. Depending on your internet source, i.e rellis wifi or KaruraLink, you may 
    need to modify the core/config.py file's ip address for the rtsp server. Unless you are zac renkema though, do
    push changes to the modified ip address to the github. I will smite you. 
    """
    frame_ready = Signal(np.ndarray)
    error_occurred = Signal(str)

    def __init__(self, parent=None, source=0):
        super().__init__(parent)
        self._is_running = True
        self.cap = None
        self.source = source  

    def run(self):
        # Decide backend based on source type
        if isinstance(self.source, str) and self.source.startswith("rtsp://"):
            # RTSP stream
            self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
            # Best-effort low latency (may not be honored on all builds)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print(f"[CameraWorker] Starting with source={self.source!r}") #DEBUG STATEMENT REMOVE LATER

        else:
            # Local camera
            self.cap = cv2.VideoCapture(int(self.source), cv2.CAP_V4L2)
            # Local camera props only
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1040)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 980)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        if not self.cap.isOpened():
            self.error_occurred.emit(f"Error: Could not open video source: {self.source}")
            self._is_running = False
            return

        while self._is_running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.frame_ready.emit(frame)
            else:
                # RTSP can drop frames briefly; don’t hard-exit immediately
                self.msleep(30)
                continue

            self.msleep(10)  # lower sleep for smoother RTSP

        self.cap.release()
        print("CameraWorker: Video capture released.")

    def stop(self):
        self._is_running = False
        self.wait()

# --- 2. Main Widget for Display ---

class VideoWidget(QWidget):
    """
    The main widget that displays the video feed.
    """
    def __init__(self, parent=None, source=0):
        super().__init__(parent)
        self.setWindowTitle("PySide6 OpenCV Camera Feed")
        # self.setMinimumSize(640, 480)

        # Layout
        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        # Label to display the frame
        self.video_label = QLabel("Waiting for camera feed...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("border: 2px solid #333; background-color: #f0f0f0;")
        self.layout.addWidget(self.video_label)

        # Initialize the worker thread (CameraWorker already IS a QThread)
        self.camera_worker = CameraWorker(self, source)

        # Connect signals from the worker thread
        self.camera_worker.frame_ready.connect(self.update_image)
        self.camera_worker.error_occurred.connect(self.handle_camera_error)
        # print(f"[CameraWorker] cap.isOpened() = {self.cap.isOpened()}")


    # Start the worker thread
    def start_camera(self):
        if not self.camera_worker.isRunning():
            self.camera_worker._is_running = True
            self.camera_worker.start()

    def stop_camera(self):
        if self.camera_worker.isRunning():
            self.camera_worker.stop()

    @Slot(np.ndarray)
    def update_image(self, cv_img):
        """
        Slot to receive the numpy array (OpenCV frame) and convert it to QPixmap.
        """
        if cv_img is None:
            return

        # Convert the BGR image from OpenCV to RGB (required for QImage)
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w

        # Create QImage from the numpy array
        convert_to_Qt_format = QImage(
            rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
        )
        
        # Convert QImage to QPixmap for display in QLabel
        pixmap = QPixmap.fromImage(convert_to_Qt_format)

        # Scale the pixmap to fit the label, maintaining aspect ratio
        # scaled_pixmap = pixmap.scaled(
        #     self.video_label.size(),
        #     Qt.KeepAspectRatio,
        #     Qt.SmoothTransformation
        # )

        # self.video_label.setPixmap(scaled_pixmap)
        self.video_label.setPixmap(pixmap)

    @Slot(str)
    def handle_camera_error(self, message):
        """
        Slot to handle and display camera errors.
        """
        print(f"CAMERA ERROR: {message}")
        QMessageBox.critical(self, "Camera Error", message)
        self.video_label.setText("Camera not available.")

    def closeEvent(self, event):
        """
        Ensure the worker thread is stopped when the main window is closed.
        """
        # if self.camera_worker and self.camera_worker.isRunning():
        #     print("Main UI: Stopping CameraWorker thread...")
            # self.camera_worker.stop()
        self.stop_camera()
        event.accept()

# --- 3. Main Application Entry Point ---

def main():
    # PySide6 application boilerplate
    app = QApplication(sys.argv)
    
    # Check for OpenCV version to ensure compatibility
    print(f"OpenCV Version: {cv2.__version__}")
    
    # Create and show the main window
    main_widget = VideoWidget(source="rtsp://172.17.248.2:8554/cam")
    main_widget.show()
    main_widget.start_camera()
    
    # Start the event loop
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
    