#rtsp_url = "rtsp://127.0.0.1:8554/test" 
rtsp_url = 0 
#!/usr/bin/env python3
import sys

import cv2
import numpy as np

from PySide6.QtCore import (
    QThread,
    Signal,
    Slot,
    QMutex,
    QMutexLocker,
    Qt,
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QLabel,
    QVBoxLayout,
    QWidget,
    QMessageBox,
)


class CameraWorker(QThread):
    """
    Captures frames from:
      - RTSP URL (OpenCV + FFMPEG)
      - Local camera index (OpenCV + V4L2)

    Supports live switching without restarting the GUI:
      - call request_source(new_source)
    """
    frame_ready = Signal(np.ndarray)
    error_occurred = Signal(str)

    def __init__(self, parent=None, source=rtsp_url):
        super().__init__(parent)
        self._is_running = True
        self.cap = None

        self._lock = QMutex()
        self._source = source
        self._switch_requested = False

    def request_source(self, new_source):
        """Request switching the capture source (RTSP URL or camera index)."""
        with QMutexLocker(self._lock):
            self._source = new_source
            self._switch_requested = True

        # Best effort to unblock a potentially-blocking cap.read() on RTSP
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

    def _open_capture(self, source):
        """Open a cv2.VideoCapture for RTSP or local camera."""
        if isinstance(source, str) and source.startswith("rtsp://"):
            #cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
            cap = cv2.VideoCapture(source)
            # Best-effort low-latency hint (not always honored)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print(f"[CameraWorker] Opening RTSP source={source!r}")
            return cap
        else:
            idx = int(source)
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1040)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 980)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            print(f"[CameraWorker] Opening local camera index={idx}")
            return cap

    def run(self):
        # Initial open
        with QMutexLocker(self._lock):
            current_source = self._source
            self._switch_requested = False

        self.cap = self._open_capture(current_source)
        if not self.cap.isOpened():
            self.error_occurred.emit(f"Error: Could not open video source: {current_source}")
            self._is_running = False
            return

        while self._is_running:
            # Handle requested source switch
            with QMutexLocker(self._lock):
                if self._switch_requested:
                    current_source = self._source
                    self._switch_requested = False

                    if self.cap is not None:
                        try:
                            self.cap.release()
                        except Exception:
                            pass

                    self.cap = self._open_capture(current_source)
                    if not self.cap.isOpened():
                        self.error_occurred.emit(f"Error: Could not open video source: {current_source}")
                        self.msleep(200)
                        continue

            # Read frame
            if self.cap is None:
                self.msleep(30)
                continue

            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.frame_ready.emit(frame)
                self.msleep(5)
            else:
                # RTSP can drop frames / reconnect; don't hard-exit
                self.msleep(30)

        # Cleanup
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        print("[CameraWorker] Video capture released.")

    def stop(self):
        """Stop the thread and release capture ASAP."""
        self._is_running = False
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.wait()


class VideoWidget(QWidget):
    """
    Widget that displays a video feed.
    Public API:
      - start_camera()
      - stop_camera()
      - switch_camera(new_source)  # RTSP url or camera index
    """

    def __init__(self, parent=None, source=rtsp_url):
        super().__init__(parent)

        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        self.video_label = QLabel("Waiting for camera feed...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("border: 2px solid #333; background-color: #f0f0f0;")
        self.layout.addWidget(self.video_label)

        self.camera_worker = CameraWorker(self, source)
        self.camera_worker.frame_ready.connect(self.update_image)
        self.camera_worker.error_occurred.connect(self.handle_camera_error)

        self._last_pixmap = None

    def start_camera(self):
        #Uncomment return to temporary disable camera for now:
        #return

        if not self.camera_worker.isRunning():
            self.camera_worker._is_running = True
            self.camera_worker.start()

    def stop_camera(self):
        if self.camera_worker.isRunning():
            self.camera_worker.stop()
        self.video_label.setText("Camera stopped.")
        self.video_label.setPixmap(QPixmap())
        self._last_pixmap = None

    def switch_camera(self, new_source):
        """
        Switch to a new RTSP URL (string) or a local index (int or numeric string).
        No app restart needed.
        """
        if not self.camera_worker.isRunning():
            self.camera_worker.request_source(new_source)
            return

        self.video_label.setText("Switching feed...")
        self.camera_worker.request_source(new_source)

    @Slot(np.ndarray)
    def update_image(self, cv_img):
        if cv_img is None:
            return

        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w

        # IMPORTANT: copy() avoids referencing temporary numpy memory
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(qimg)

        self._last_pixmap = pixmap
        scaled = pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video_label.setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._last_pixmap is not None and not self._last_pixmap.isNull():
            scaled = self._last_pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(scaled)

    @Slot(str)
    def handle_camera_error(self, message):
        print(f"CAMERA ERROR: {message}")
        QMessageBox.critical(self, "Camera Error", message)
        self.video_label.setText("Camera not available.")
        self.video_label.setPixmap(QPixmap())
        self._last_pixmap = None

    def closeEvent(self, event):
        self.stop_camera()
        event.accept()
