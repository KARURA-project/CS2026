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
    A QThread subclass to handle the OpenCV video capture in a separate thread.
    This prevents the main UI thread from freezing.
    """
    frame_ready = Signal(np.ndarray) # Signal to emit the captured frame (numpy array)
    error_occurred = Signal(str)     # Signal for errors, like camera not opening

    def __init__(self, parent=None, camera_id = 0):
        super().__init__(parent)
        self._is_running = True
        self.cap = None
        self.camera_id = camera_id

    def run(self):
        """
        The main loop of the thread, where video capture happens.
        """
        # 0 usually refers to the first camera. Using CAP_V4L2 for Linux compatibility
        # as suggested by the original code, but it's often optional.
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.error_occurred.emit("Error: Could not open camera. Check camera index or permissions.")
            self._is_running = False
            return

        # Set properties (as per original request)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # Setting MJPG is good practice for performance, though not always required
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        while self._is_running:
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            if ret:
                # Emit the captured frame (numpy array) to the main thread
                self.frame_ready.emit(frame)
            else:
                # Handle frame read error
                print("Warning: Could not read frame.")
                break # Exit loop on read error

            # Control frame rate roughly (like the original time.sleep(.100))
            # QThread provides better ways, but a brief wait is fine for this example.
            self.msleep(30) # Roughly 33 FPS (1000ms / 30ms ≈ 33 FPS)

        # Cleanup when the loop exits
        self.cap.release()
        print("CameraWorker: Video capture released.")

    def stop(self):
        """
        Gracefully stop the thread loop.
        """
        self._is_running = False
        self.wait() # Wait for the thread to finish execution

# --- 2. Main Widget for Display ---

class VideoWidget(QWidget):
    """
    The main widget that displays the video feed.
    """
    def __init__(self, parent=None, camera_id = 0):
        super().__init__()
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

        # Initialize the worker thread
        self.camera_worker = CameraWorker(None, camera_id)

        # Connect signals from the worker thread
        self.camera_worker.frame_ready.connect(self.update_image)
        self.camera_worker.error_occurred.connect(self.handle_camera_error)

        # Start the worker thread
        self.camera_worker.start()
        print("Main UI: CameraWorker thread started.")

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
        if self.camera_worker and self.camera_worker.isRunning():
            print("Main UI: Stopping CameraWorker thread...")
            self.camera_worker.stop()
        event.accept()

# --- 3. Main Application Entry Point ---

if __name__ == '__main__':
    # PySide6 application boilerplate
    app = QApplication(sys.argv)
    
    # Check for OpenCV version to ensure compatibility
    print(f"OpenCV Version: {cv2.__version__}")
    
    # Create and show the main window
    main_widget = VideoWidget()
    main_widget.show()
    
    # Start the event loop
    sys.exit(app.exec())