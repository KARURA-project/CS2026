import rclpy
from PySide6.QtCore import Signal

from dashboard.backend.mobility_node import MobilityNode
from dashboard.backend.ps4_teleop_node import PS4TeleopNode
from dashboard.core.base_bridge import BaseROS2Bridge

from std_msgs.msg import String, Float64MultiArray, Float64, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class MobilityBridge(BaseROS2Bridge):
    mobility_status_signal = Signal(String)
    actual_rads_signal = Signal(Float64MultiArray)
    roll_pitch_yaw_signal = Signal(Float64MultiArray)
    odometry_filtered_signal = Signal(Odometry)
    bandwidth_signal = Signal(Float64)
    battery_data_signal = Signal(Int32)
    battery_voltage_data_signal = Signal(Float64)
    battery_power_data_signal = Signal(Float64)
    navigation_status_signal = Signal(String)
    fisheye_cam_signal = Signal(String)
    gps_map_signal = Signal(Float64MultiArray)
    gps_data_signal = Signal(NavSatFix)

    teleop_cmd_signal = Signal(float, float)
    error_signal = Signal(str)

    def __init__(self,
                 mobility_node_name: str = "karura_mobility_gui",
                 teleop_node_name: str = "karura_ps4_teleop"):

        # Init ROS BEFORE creating nodes
        if not rclpy.ok():
            rclpy.init(args=None)

        # Create BOTH nodes
        self.mobility_node = MobilityNode(mobility_node_name)
        self.teleop_node = PS4TeleopNode(teleop_node_name)

        # IMPORTANT: initialize BaseROS2Bridge/QObject BEFORE touching signals like self.ros_error
        super().__init__([self.mobility_node, self.teleop_node])

        # Connect ros_error ONCE, AFTER super()
        self.ros_error.connect(self._on_ros_error)

        # Mobility telemetry callbacks
        self.mobility_node.register_callback("mobility_status", self._emit_mobility_status)
        self.mobility_node.register_callback("actual_rads", self._emit_actual_rads)
        self.mobility_node.register_callback("roll_pitch_yaw", self._emit_roll_pitch_yaw)
        self.mobility_node.register_callback("odometry_filtered", self._emit_odometry_filtered)
        self.mobility_node.register_callback("bandwidth", self._emit_bandwidth)
        self.mobility_node.register_callback("battery_data", self._emit_battery_data)
        self.mobility_node.register_callback("battery_voltage_data", self._emit_battery_voltage_data)
        self.mobility_node.register_callback("battery_power_data", self._emit_battery_power_data)
        self.mobility_node.register_callback("navigation_status", self._emit_navigation_status)
        self.mobility_node.register_callback("fisheye_cam", self._emit_fisheye_cam)
        self.mobility_node.register_callback("gps_map", self._emit_gps_map)
        self.mobility_node.register_callback("gps_data", self._emit_gps_data)

        # Teleop callback (optional for UI)
        self.teleop_node.register_callback("teleop_cmd", self._emit_teleop_cmd)


        # forward ROS worker errors
        self.ros_error.connect(self._on_ros_error)

    # GUI controls for teleop 
    def set_teleop_enabled(self, enabled: bool):
        self.teleop_node.set_enabled(enabled)

    def estop(self):
        self.teleop_node.trigger_estop()

    def clear_estop(self):
        self.teleop_node.clear_estop()

    # ---- Emitters ----
    def _emit_mobility_status(self, msg: String):
        try:
            self.mobility_status_signal.emit(msg)
        except Exception as e:
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

    def _emit_battery_voltage_data(self, msg: Float64):
        try:
            self.battery_voltage_data_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting battery_voltage_data")
        
    def _emit_battery_power_data(self, msg: Float64):
        try:
            self.battery_power_data_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting battery_power_data")
        

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

    def _emit_teleop_cmd(self, data):
        try:
            self.teleop_cmd_signal.emit(float(data["linear"]), float(data["angular"]))
        except Exception as e:
            self.error_signal.emit(f"Error emitting teleop_cmd: {e}")

    def _on_ros_error(self, error_msg: str):
        self.error_signal.emit(f"ROS 2 Error: {error_msg}")
