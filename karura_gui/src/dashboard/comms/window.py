import sys

from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QSplitter,
    QVBoxLayout,
    QWidget,
    QGroupBox,
    QGridLayout,
    QProgressBar,
    QCheckBox,
    QHBoxLayout,
)
from PySide6.QtCore import Qt


class BandwidthMonitorView(QWidget):
    """
    Top-left view: Bandwidth Monitor + Usage Meter + Camera usage (view only).
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(10)

        # Section title
        title = QLabel("Bandwidth Monitor")
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        main_layout.addWidget(title)

        # ------------------ LEFT: link & total bandwidth ------------------
        left_group = QGroupBox("Link & Bandwidth")
        left_grid = QGridLayout(left_group)
        left_grid.setContentsMargins(8, 8, 8, 8)
        left_grid.setHorizontalSpacing(10)
        left_grid.setVerticalSpacing(10)

        # Row 0: Status
        lbl_status = QLabel("Status:")
        self.val_status = QLabel("OFFLINE")  # placeholder
        left_grid.addWidget(lbl_status, 0, 0)
        left_grid.addWidget(self.val_status, 0, 1)

        # Row 1: Total Bandwidth
        lbl_total_bw = QLabel("Total Bandwidth:")
        self.val_total_bw = QLabel("--- Mbps")
        left_grid.addWidget(lbl_total_bw, 1, 0)
        left_grid.addWidget(self.val_total_bw, 1, 1)

        # Row 2: Camera Feed Usage (aggregate text)
        lbl_cam_usage = QLabel("Camera Feed Usage:")
        self.val_cam_usage = QLabel("--- Mbps")
        left_grid.addWidget(lbl_cam_usage, 2, 0)
        left_grid.addWidget(self.val_cam_usage, 2, 1)

        # Row 3: Usage meter (progress bar)
        lbl_usage_meter = QLabel("Usage Meter:")
        self.usage_bar = QProgressBar()
        self.usage_bar.setRange(0, 100)  # 0–100%
        self.usage_bar.setValue(0)
        left_grid.addWidget(lbl_usage_meter, 3, 0)
        left_grid.addWidget(self.usage_bar, 3, 1)

        # ------------------ RIGHT: per-camera controls ------------------
        cam_group = QGroupBox("Camera Feeds")
        cam_grid = QGridLayout(cam_group)
        cam_grid.setContentsMargins(8, 8, 8, 8)
        cam_grid.setHorizontalSpacing(10)
        cam_grid.setVerticalSpacing(8)

        # You can rename these to match exactly what's in the PDF
        camera_rows = [
            ("Front Cam", "front"),
            ("Rear Cam", "rear"),
            ("Arm Cam", "arm"),
            ("Science Cam", "science"),
        ]

        self.cam_toggles = {}
        self.cam_usage_labels = {}

        for row, (label_text, key) in enumerate(camera_rows):
            lbl_cam = QLabel(label_text)

            toggle = QCheckBox("On")
            toggle.setChecked(False)  # default off

            usage_lbl = QLabel("-- Mbps")  # per-camera bandwidth usage

            cam_grid.addWidget(lbl_cam, row, 0)
            cam_grid.addWidget(toggle, row, 1)
            cam_grid.addWidget(usage_lbl, row, 2)

            self.cam_toggles[key] = toggle
            self.cam_usage_labels[key] = usage_lbl

        # ------------------ Combine left + right into one row ------------------
        content_layout = QHBoxLayout()
        content_layout.setSpacing(16)

        content_layout.addWidget(left_group)

        # vertical line between the two groups (for visual division)
        divider = QWidget()
        divider.setFixedWidth(1)
        divider.setStyleSheet("background-color: #555;")
        content_layout.addWidget(divider)

        content_layout.addWidget(cam_group)

        main_layout.addLayout(content_layout)
        main_layout.addStretch()


class CameraManagementView(QWidget):
    """
    Top-right view: Camera Management (placeholder for now, you can extend later).
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(10)

        title = QLabel("Camera Management")
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        main_layout.addWidget(title)

        placeholder = QLabel("Camera Management Controls Here")
        placeholder.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(placeholder)

        main_layout.addStretch()


class CommsWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Communication Status")

        # ---- Central widget + root layout (QMainWindow needs this) ----
        central = QWidget(self)
        self.setCentralWidget(central)

        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # ---- Vertical splitter: top (row) / bottom (row) ----
        vertical_splitter = QSplitter(Qt.Orientation.Vertical)

        # ---- Top row: left/right ----
        top_horizontal_splitter = QSplitter(Qt.Orientation.Horizontal)

        # TOP-LEFT: Bandwidth Monitor view (now with camera usage on the right)
        top_left = BandwidthMonitorView()

        # TOP-RIGHT: for now, placeholder (e.g., cFS logs)
        top_right = QLabel("Top Right (cFS Logs)")
        top_right.setAlignment(Qt.AlignCenter)

        top_horizontal_splitter.addWidget(top_left)
        top_horizontal_splitter.addWidget(top_right)

        # ---- Bottom row: left/right ----
        bottom_horizontal_splitter = QSplitter(Qt.Orientation.Horizontal)

        bottom_left = QLabel("Bottom Left")
        bottom_left.setAlignment(Qt.AlignCenter)

        bottom_right = QLabel("Bottom Right")
        bottom_right.setAlignment(Qt.AlignCenter)

        bottom_horizontal_splitter.addWidget(bottom_left)
        bottom_horizontal_splitter.addWidget(bottom_right)

        # Assemble vertical splitter
        vertical_splitter.addWidget(top_horizontal_splitter)
        vertical_splitter.addWidget(bottom_horizontal_splitter)

        # Add to root layout
        root_layout.addWidget(vertical_splitter)

        # ---- Make the quadrant divisions visually obvious ----
        splitter_style = """
        QSplitter::handle {
            background-color: #555555;
        }
        QSplitter::handle:horizontal {
            width: 2px;
        }
        QSplitter::handle:vertical {
            height: 2px;
        }
        """
        self.setStyleSheet(splitter_style)

        # Optional initial sizes
        vertical_splitter.setSizes([250, 250])            # top vs bottom
        top_horizontal_splitter.setSizes([400, 400])      # top-left vs top-right
        bottom_horizontal_splitter.setSizes([400, 400])   # bottom-left vs bottom-right


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = CommsWindow()
    win.show()
    sys.exit(app.exec())
