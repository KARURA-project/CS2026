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
    QFrame,
)
from PySide6.QtCore import Qt, QTimer
from dashboard.core.widgets.bottom_bar import BottomBarView

class BandwidthMonitorView(QWidget):
    """
    Top-left view: Bandwidth Monitor on top,
    Camera toggles + per-camera bandwidth underneath (view only).
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(10)

        # ----- Section title -----
        title = QLabel("Bandwidth Monitor")
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        main_layout.addWidget(title)

        # ================= TOP: Link & Total Bandwidth =================
        link_group = QGroupBox("Link & Bandwidth")
        link_grid = QGridLayout(link_group)
        link_grid.setContentsMargins(8, 8, 8, 8)
        link_grid.setHorizontalSpacing(10)
        link_grid.setVerticalSpacing(8)

        # Row 0: Status
        lbl_status = QLabel("Status:")
        self.val_status = QLabel("OFFLINE")  # placeholder
        link_grid.addWidget(lbl_status, 0, 0)
        link_grid.addWidget(self.val_status, 0, 1)

        # Row 1: Total Bandwidth
        lbl_total_bw = QLabel("Total Bandwidth:")
        self.val_total_bw = QLabel("--- Mbps")
        link_grid.addWidget(lbl_total_bw, 1, 0)
        link_grid.addWidget(self.val_total_bw, 1, 1)

        # Row 2: Camera Feed Usage (aggregate text)
        lbl_cam_usage = QLabel("Camera Feed Usage:")
        self.val_cam_usage = QLabel("--- Mbps")
        link_grid.addWidget(lbl_cam_usage, 2, 0)
        link_grid.addWidget(self.val_cam_usage, 2, 1)

        # Row 3: Usage meter (progress bar)
        lbl_usage_meter = QLabel("Usage Meter:")
        self.usage_bar = QProgressBar()
        self.usage_bar.setRange(0, 100)  # 0–100%
        self.usage_bar.setValue(0)
        link_grid.addWidget(lbl_usage_meter, 3, 0)
        link_grid.addWidget(self.usage_bar, 3, 1)

        main_layout.addWidget(link_group)

        # ----- Thin separator line between top and bottom parts -----
        divider = QFrame()
        divider.setFrameShape(QFrame.HLine)
        divider.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(divider)

        # ================= BOTTOM: Per-camera controls =================
        cam_group = QGroupBox("Camera Feeds")
        cam_grid = QGridLayout(cam_group)
        cam_grid.setContentsMargins(8, 8, 8, 8)
        cam_grid.setHorizontalSpacing(10)
        cam_grid.setVerticalSpacing(8)

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
            toggle.setChecked(False)

            usage_lbl = QLabel("-- Mbps")

            cam_grid.addWidget(lbl_cam, row, 0)
            cam_grid.addWidget(toggle, row, 1)
            cam_grid.addWidget(usage_lbl, row, 2)

            self.cam_toggles[key] = toggle
            self.cam_usage_labels[key] = usage_lbl

        main_layout.addWidget(cam_group)
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

        # ---- Vertical splitter: top row / bottom row ----
        self.vertical_splitter = QSplitter(Qt.Orientation.Vertical)

        # ---- Top row: left/right ----
        self.top_horizontal_splitter = QSplitter(Qt.Orientation.Horizontal)

        # TOP-LEFT: Bandwidth + Cameras (stacked)
        top_left = BandwidthMonitorView()

        # TOP-RIGHT: placeholder (e.g., cFS Logs)
        top_right = QLabel("Top Right (cFS Logs)")
        top_right.setAlignment(Qt.AlignCenter)

        self.top_horizontal_splitter.addWidget(top_left)
        self.top_horizontal_splitter.addWidget(top_right)

        # ---- Bottom row: left/right ----
        self.bottom_horizontal_splitter = QSplitter(Qt.Orientation.Horizontal)

        bottom_left = QLabel("Bottom Left")
        bottom_left.setAlignment(Qt.AlignCenter)

        bottom_right = QLabel("Bottom Right")
        bottom_right.setAlignment(Qt.AlignCenter)

        self.bottom_horizontal_splitter.addWidget(bottom_left)
        self.bottom_horizontal_splitter.addWidget(bottom_right)

        # Assemble vertical splitter
        self.vertical_splitter.addWidget(self.top_horizontal_splitter)
        self.vertical_splitter.addWidget(self.bottom_horizontal_splitter)

        # Add to root layout
        root_layout.addWidget(self.vertical_splitter)

        # Add shared bottom bar
        self.bottom_bar = BottomBarView(self)
        root_layout.addWidget(self.bottom_bar)

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

        # Set equal stretch factors (helps keep things balanced on resize)
        self.vertical_splitter.setStretchFactor(0, 1)
        self.vertical_splitter.setStretchFactor(1, 1)
        self.top_horizontal_splitter.setStretchFactor(0, 1)
        self.top_horizontal_splitter.setStretchFactor(1, 1)
        self.bottom_horizontal_splitter.setStretchFactor(0, 1)
        self.bottom_horizontal_splitter.setStretchFactor(1, 1)

        # IMPORTANT: set initial sizes AFTER the window is laid out
        QTimer.singleShot(0, self._set_initial_split_sizes)

    def _set_initial_split_sizes(self):
        """Force an even 2x2 split once the window has a real size."""
        h = max(self.vertical_splitter.height(), 1)
        w_top = max(self.top_horizontal_splitter.width(), 1)
        w_bottom = max(self.bottom_horizontal_splitter.width(), 1)

        # Top vs bottom
        self.vertical_splitter.setSizes([h // 2, h // 2])
        # Left vs right (top row)
        self.top_horizontal_splitter.setSizes([w_top // 2, w_top // 2])
        # Left vs right (bottom row)
        self.bottom_horizontal_splitter.setSizes([w_bottom // 2, w_bottom // 2])


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = CommsWindow()
    win.show()
    sys.exit(app.exec())
