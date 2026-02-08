"""
Main entry point for the Arm Dashboard GUI.

This module bootstraps the Arm dashboard application with:
- PySide6 Qt application
- ROS 2 ArmNode backend
- Qt/ROS 2 bridge for topic subscriptions
- Main window UI with arm controls and telemetry
"""

import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, 
    QLabel, QGroupBox, QSlider, QPushButton, QSpinBox, QDoubleSpinBox,
    QFrame, QGridLayout, QComboBox
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont

from karura_gui.backend.arm_node import ArmNode
from karura_gui.core.base_bridge import BaseROS2Bridge


class TelemetryWidget(QGroupBox):
    """Widget to display telemetry data in a vertical layout."""
    
    def __init__(self, title: str = "Telemetry"):
        super().__init__(title)
        layout = QVBoxLayout()
        
        # Telemetry items: battery, connection status, temperature, etc.
        self.telemetry_items = {}
        
        # Battery Status
        self._add_telemetry_item("battery_voltage", "Battery Voltage:", "N/A V", layout)
        self._add_telemetry_item("battery_current", "Battery Current:", "N/A A", layout)
        self._add_telemetry_item("battery_percentage", "Battery %:", "N/A %", layout)
        
        layout.addSpacing(10)
        
        # Connection Status
        self._add_telemetry_item("connection_status", "Connection:", "Disconnected", layout)
        self._add_telemetry_item("signal_strength", "Signal Strength:", "N/A dBm", layout)
        
        layout.addSpacing(10)
        
        # Temperature
        self._add_telemetry_item("arm_temperature", "Arm Temp:", "N/A °C", layout)
        self._add_telemetry_item("motor_temperature", "Motor Temp:", "N/A °C", layout)
        
        layout.addSpacing(10)
        
        # Arm Status
        self._add_telemetry_item("arm_status", "Arm Status:", "Idle", layout)
        self._add_telemetry_item("gripper_status", "Gripper:", "Open", layout)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _add_telemetry_item(self, key: str, label: str, default: str, layout):
        """Helper to add a telemetry display item."""
        h_layout = QHBoxLayout()
        label_widget = QLabel(label)
        label_widget.setMinimumWidth(120)
        label_widget.setFont(QFont("Arial", 9, QFont.Bold))
        
        value_widget = QLabel(default)
        value_widget.setMinimumWidth(100)
        value_widget.setStyleSheet("border: 1px solid #ccc; padding: 2px;")
        
        h_layout.addWidget(label_widget)
        h_layout.addWidget(value_widget)
        layout.addLayout(h_layout)
        
        self.telemetry_items[key] = value_widget
    
    def update_telemetry(self, key: str, value: str):
        """Update a telemetry value."""
        if key in self.telemetry_items:
            self.telemetry_items[key].setText(value)


class JointControlWidget(QGroupBox):
    """Widget for controlling individual arm joints."""
    
    def __init__(self, joint_name: str):
        super().__init__(f"{joint_name} Control")
        layout = QGridLayout()
        
        # Slider for position control
        layout.addWidget(QLabel("Position:"), 0, 0)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-180, 180)
        slider.setValue(0)
        layout.addWidget(slider, 0, 1)
        
        self.position_label = QLabel("0°")
        layout.addWidget(self.position_label, 0, 2)
        slider.valueChanged.connect(lambda v: self.position_label.setText(f"{v}°"))
        
        # Speed control
        layout.addWidget(QLabel("Speed:"), 1, 0)
        speed_spin = QSpinBox()
        speed_spin.setRange(0, 100)
        speed_spin.setValue(50)
        speed_spin.setSuffix("%")
        layout.addWidget(speed_spin, 1, 1)
        
        # Current display
        layout.addWidget(QLabel("Current:"), 2, 0)
        self.current_label = QLabel("0 mA")
        layout.addWidget(self.current_label, 2, 1)
        
        # Status
        layout.addWidget(QLabel("Status:"), 3, 0)
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: green;")
        layout.addWidget(self.status_label, 3, 1)
        
        self.setLayout(layout)


class GripperControlWidget(QGroupBox):
    """Widget for gripper control and feedback."""
    
    def __init__(self):
        super().__init__("Gripper Control")
        layout = QGridLayout()
        
        # Gripper position
        layout.addWidget(QLabel("Position:"), 0, 0)
        gripper_spin = QDoubleSpinBox()
        gripper_spin.setRange(0.0, 100.0)
        gripper_spin.setValue(0.0)
        gripper_spin.setSuffix("%")
        layout.addWidget(gripper_spin, 0, 1)
        
        # Quick buttons
        open_btn = QPushButton("Open")
        close_btn = QPushButton("Close")
        layout.addWidget(open_btn, 1, 0)
        layout.addWidget(close_btn, 1, 1)
        
        # Status
        layout.addWidget(QLabel("Status:"), 2, 0)
        self.gripper_status = QLabel("Idle")
        layout.addWidget(self.gripper_status, 2, 1)
        
        self.setLayout(layout)


class ArmMainWindow(QMainWindow):
    """
    Main window for the Arm dashboard.
    
    Features:
    - Joint state displays and controls (6 DOF arm)
    - Gripper control
    - Telemetry sidebar (battery, temperature, connection status)
    - Camera view placeholder
    """

    def __init__(self, bridge: BaseROS2Bridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("Karura Arm Dashboard")
        self.setGeometry(100, 100, 1400, 800)

        # Create main layout
        main_widget = QWidget()
        main_layout = QHBoxLayout(main_widget)
        
        # Left side: Arm controls
        left_panel = self._create_control_panel()
        
        # Right side: Telemetry
        self.telemetry = TelemetryWidget("System Telemetry")
        self.telemetry.setMinimumWidth(250)
        self.telemetry.setMaximumWidth(300)
        
        main_layout.addWidget(left_panel, 2)
        main_layout.addWidget(self.telemetry, 1)
        
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)
    
    def _create_control_panel(self) -> QWidget:
        """Create the arm control panel with all joints and gripper."""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Arm Control System")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(title)
        
        # Create joint controls (6 DOF)
        joint_names = ["Joint 1 (Base)", "Joint 2 (Shoulder)", "Joint 3 (Elbow)", 
                       "Joint 4 (Wrist 1)", "Joint 5 (Wrist 2)", "Joint 6 (Wrist 3)"]
        
        self.joint_widgets = []
        for joint_name in joint_names:
            joint_widget = JointControlWidget(joint_name)
            self.joint_widgets.append(joint_widget)
            layout.addWidget(joint_widget)
        
        layout.addSpacing(10)
        
        # Gripper control
        self.gripper = GripperControlWidget()
        layout.addWidget(self.gripper)
        
        layout.addSpacing(10)
        
        # Mode selection
        mode_layout = QGridLayout()
        mode_layout.addWidget(QLabel("Control Mode:"), 0, 0)
        mode_combo = QComboBox()
        mode_combo.addItems(["Joint Control", "IK Control", "Teach Mode"])
        mode_layout.addWidget(mode_combo, 0, 1)
        layout.addLayout(mode_layout)
        
        # Action buttons
        button_layout = QHBoxLayout()
        home_btn = QPushButton("Home")
        home_btn.setMinimumWidth(80)
        reset_btn = QPushButton("Reset")
        reset_btn.setMinimumWidth(80)
        emergency_btn = QPushButton("E-STOP")
        emergency_btn.setStyleSheet("background-color: #ff4444; color: white; font-weight: bold;")
        emergency_btn.setMinimumWidth(80)
        
        button_layout.addWidget(home_btn)
        button_layout.addWidget(reset_btn)
        button_layout.addWidget(emergency_btn)
        layout.addLayout(button_layout)
        
        layout.addStretch()
        panel.setLayout(layout)
        return panel

    def closeEvent(self, event):
        """Shutdown ROS 2 bridge on window close."""
        self.bridge.shutdown()
        event.accept()


def main():
    """Initialize and run the Arm Dashboard."""
    app = QApplication(sys.argv)

    # Create the ROS 2 bridge with ArmNode
    bridge = BaseROS2Bridge(ArmNode, "karura_arm_gui")
    bridge.start()

    # Create and show the main window
    window = ArmMainWindow(bridge)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
