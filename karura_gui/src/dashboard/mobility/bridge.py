#TODO - this should connect the frontend and the backend.
from typing import Text

from PySide6.QtCore import QObject, Signal

from dashboard.backend.mobility_node import MobilityNode
from dashboard.core.base_bridge import BaseROS2Bridge

from std_msgs.msg import String, Float64MultiArray, Float64, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class MobilityBridge(BaseROS2Bridge):

    mobility_status_signal = Signal(String)
    actual_rads_signal = Signal(Float64MultiArray)
    roll_pitch_yaw_signal = Signal(Float64MultiArray)
    odometry_filtered_signal = Signal(Odometry)
    bandwidth_signal = Signal(Float64)
    battery_data_signal = Signal(Int32)
    navigation_status_signal = Signal(String)
    fisheye_cam_signal = Signal(String)
    gps_map_signal = Signal(Float64MultiArray)
    gps_data_signal = Signal(NavSatFix)

    error_signal = Signal(str)

    def __init__(self, node_name: str = "mobility_bridge_node"):
        super().__init__(MobilityNode, node_name)
        self.mobility_node: MobilityNode = self.node

        # Register callbacks 
        self.mobility_node.register_callback("mobility_status", self._emit_mobility_status)
        self.mobility_node.register_callback("actual_rads", self._emit_actual_rads)
        self.mobility_node.register_callback("roll_pitch_yaw", self._emit_roll_pitch_yaw)
        self.mobility_node.register_callback("odometry_filtered", self._emit_odometry_filtered)
        self.mobility_node.register_callback("bandwidth", self._emit_bandwidth)
        self.mobility_node.register_callback("battery_data", self._emit_battery_data)
        self.mobility_node.register_callback("navigation_status", self._emit_navigation_status)
        self.mobility_node.register_callback("fisheye_cam", self._emit_fisheye_cam)
        self.mobility_node.register_callback("gps_map", self._emit_gps_map)
        self.mobility_node.register_callback("gps_data", self._emit_gps_data)

        self.ros_error.connect(self._on_ros_error)

    
    def _emit_mobility_status(self, msg: String):
        try:
            print("[Bridge] mobility_status:", getattr(msg, "data", msg))
            self.mobility_status_signal.emit(msg)
        except Exception as e:
            print("[Bridge] ERROR mobility_status: ", e)
            self.error_signal.emit(f"Error emitting mobility_status: {e}")

    def _emit_actual_rads(self, msg: Float64MultiArray):
        try:
            self.actual_rads_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting actual_rads: {e}")

    def _emit_roll_pitch_yaw(self, msg: Float64MultiArray):
        try:
            self.roll_pitch_yaw_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting roll_pitch_yaw: {e}")

    def _emit_odometry_filtered(self, msg: Odometry):
        try:
            self.odometry_filtered_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting odometry_filtered: {e}")

    def _emit_bandwidth(self, msg: Float64):
        try:
            self.bandwidth_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting bandwidth: {e}")

    def _emit_battery_data(self, msg: Int32):
        try:
            self.battery_data_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting battery_data: {e}")

    def _emit_navigation_status(self, msg: String):
        try:
            self.navigation_status_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting navigation_status: {e}")

    def _emit_fisheye_cam(self, msg: String):
        try:
            self.fisheye_cam_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting fisheye_cam: {e}")

    def _emit_gps_map(self, msg: Float64MultiArray):
        try:
            self.gps_map_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting gps_map: {e}")

    def _emit_gps_data(self, msg: NavSatFix):
        try:
            self.gps_data_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting gps_data: {e}")

    def _on_ros_error(self, error_msg: str):
        self.error_signal.emit(f"ROS 2 Error: {error_msg}")
        