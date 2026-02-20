"""
Arm Dashboard Main Window

Layout:
  ┌─────────────────┬─────────────────┐
  │   Camera 1      │   Camera 2A     │
  │   (Top Left)    │   (Top Right)   │
  ├─────────────────┼─────────────────┤
  │   Camera 2B     │   Arm Controls  │
  │   (Bottom Left) │  (Bottom Right) │
  └─────────────────┴─────────────────┘
"""

import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QGroupBox,
    QSlider, QPushButton, QSpinBox, QDoubleSpinBox,
    QGridLayout, QHBoxLayout, QVBoxLayout, QComboBox,
    QScrollArea, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont
from pathlib import Path

import sys
import os

# Allow importing VideoWidget from mobility's custom_widgets
MOBILITY_DIR = os.path.join(os.path.dirname(__file__), '..', 'mobility')
sys.path.insert(0, MOBILITY_DIR)

try:
    from custom_widgets.Camera import VideoWidget
    HAS_VIDEO = True
except ImportError:
    HAS_VIDEO = False

from dashboard.backend.arm_node import ArmNode
from dashboard.core.base_bridge import BaseROS2Bridge


# ---------------------------------------------------------------------------
# Camera panel helper
# ---------------------------------------------------------------------------

class CameraPanel(QFrame):
    """A labelled camera feed panel. Falls back to a placeholder if VideoWidget is unavailable."""

    def __init__(self, label: str, camera_index: int, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setFrameShadow(QFrame.Shadow.Raised)
        #self.setStyleSheet("background-color: #1a1a1a;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Label at top
        title = QLabel(label)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        #title.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        layout.addWidget(title)

        if HAS_VIDEO:
            self.video = VideoWidget(None, camera_index)
            self.video.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            layout.addWidget(self.video)
        else:
            placeholder = QLabel(f"Camera {camera_index + 1}\n(VideoWidget unavailable)")
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            #placeholder.setStyleSheet("color: #555555; font-size: 13px;")
            placeholder.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            layout.addWidget(placeholder)


# ---------------------------------------------------------------------------
# Arm control widgets
# ---------------------------------------------------------------------------


class JointControlWidget(QGroupBox):
    """Single joint: position slider, speed spinner, current readout, status."""

    def __init__(self, joint_name: str, parent=None):
        super().__init__(joint_name, parent)
        layout = QGridLayout(self)
        layout.setSpacing(4)

        # Position slider
        layout.addWidget(QLabel("Pos:"), 0, 0)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(-180, 180)
        self.slider.setValue(0)
        layout.addWidget(self.slider, 0, 1)
        self.pos_label = QLabel("0°")
        self.pos_label.setMinimumWidth(32)
        layout.addWidget(self.pos_label, 0, 2)
        self.slider.valueChanged.connect(lambda v: self.pos_label.setText(f"{v}°"))

        # Speed
        layout.addWidget(QLabel("Speed:"), 1, 0)
        speed = QSpinBox()
        speed.setRange(0, 100)
        speed.setValue(50)
        speed.setSuffix("%")
        layout.addWidget(speed, 1, 1)

        # Current
        layout.addWidget(QLabel("Current:"), 2, 0)
        self.current_label = QLabel("0 mA")
        layout.addWidget(self.current_label, 2, 1)

        # Status
        layout.addWidget(QLabel("Status:"), 3, 0)
        self.status_label = QLabel("Ready")
        #self.status_label.setStyleSheet("color: green;")
        layout.addWidget(self.status_label, 3, 1)


class GripperControlWidget(QGroupBox):
    """Gripper: position spinbox, open/close buttons, status."""

    def __init__(self, parent=None):
        super().__init__("Gripper", parent)
        layout = QGridLayout(self)
        layout.setSpacing(4)

        layout.addWidget(QLabel("Position:"), 0, 0)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(0.0, 100.0)
        self.spin.setSuffix("%")
        layout.addWidget(self.spin, 0, 1)

        self.open_btn = QPushButton("Open")
        self.close_btn = QPushButton("Close")
        layout.addWidget(self.open_btn, 1, 0)
        layout.addWidget(self.close_btn, 1, 1)

        layout.addWidget(QLabel("Status:"), 2, 0)
        self.status_label = QLabel("Idle")
        layout.addWidget(self.status_label, 2, 1)


class ArmControlsPanel(QScrollArea):
    """
    Scrollable panel containing all arm controls:
    joint sliders, gripper, mode selector, and action buttons.
    Lives in the bottom-right quadrant.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(6)
        layout.setContentsMargins(6, 6, 6, 6)



        # Joint controls
        joint_names = [
            "Joint 1 — Base",
            "Joint 2 — Shoulder",
            "Joint 3 — Elbow",
            "Joint 4 — Wrist 1",
            "Joint 5 — Wrist 2",
            "Joint 6 — Wrist 3",
        ]
        self.joint_widgets: list[JointControlWidget] = []
        for name in joint_names:
            w = JointControlWidget(name)
            self.joint_widgets.append(w)
            layout.addWidget(w)

        # Gripper
        self.gripper = GripperControlWidget()
        layout.addWidget(self.gripper)

        # Mode selector
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Joint Control", "IK Control", "Teach Mode"])
        mode_row.addWidget(self.mode_combo)
        layout.addLayout(mode_row)

        # Action buttons
        btn_row = QHBoxLayout()
        self.home_btn = QPushButton("Home")
        self.reset_btn = QPushButton("Reset")
        self.estop_btn = QPushButton("E-STOP")
        # self.estop_btn.setStyleSheet(
        #     "background-color: #ff4444; color: white; font-weight: bold;"
        # )
        for btn in (self.home_btn, self.reset_btn, self.estop_btn):
            btn.setMinimumWidth(70)
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)

        layout.addStretch()
        self.setWidget(container)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class ArmMainWindow(QMainWindow):
    """
    Arm dashboard — four-quadrant layout:

        ┌──────────┬──────────┐
        │  Cam 1   │  Cam 2A  │
        ├──────────┼──────────┤
        │  Cam 2B  │ Controls │
        └──────────┴──────────┘
    """

    def __init__(self, bridge: BaseROS2Bridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("Karura Arm Dashboard")
        self.setMinimumSize(QSize(1024, 768))

        root = QWidget()
        grid = QGridLayout(root)
        grid.setSpacing(4)
        grid.setContentsMargins(6, 6, 6, 6)

        # Equal row and column stretch so all quadrants resize together
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        # Top-left: Camera 1
        self.cam1 = CameraPanel("Camera 1", camera_index=0)
        grid.addWidget(self.cam1, 0, 0)

        # Top-right: Camera 2 — angle A
        self.cam2a = CameraPanel("Camera 2 — Angle A", camera_index=1)
        grid.addWidget(self.cam2a, 0, 1)

        # Bottom-left: Camera 2 — angle B
        self.cam2b = CameraPanel("Camera 2 — Angle B", camera_index=2)
        grid.addWidget(self.cam2b, 1, 0)

        # Bottom-right: Arm controls
        self.controls = ArmControlsPanel()
        grid.addWidget(self.controls, 1, 1)

        self.setCentralWidget(root)

    def closeEvent(self, event):
        self.bridge.shutdown()
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    
    app = QApplication(sys.argv)

    #Style Sheet
    qss_path = Path(__file__).resolve().parent / ".."/ "core" / "karura_dark.qss"
    app.setStyleSheet(qss_path.read_text(encoding="utf-8"))
    print(f"[STYLE] Loaded QSS: {qss_path}", file=sys.stderr, flush=True)

    bridge = BaseROS2Bridge(ArmNode, "karura_arm_gui")
    bridge.start()

    window = ArmMainWindow(bridge)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()