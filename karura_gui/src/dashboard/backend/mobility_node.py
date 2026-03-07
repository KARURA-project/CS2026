"""
MobilityNode

ROS 2 node responsible for mobility-related topics for the Karura dashboard GUI.

TODO: fill in publishers/subscribers for mobility-only topics
using the BaseDashboardNode._dispatch() pattern.
"""
import sys
import rclpy
from rclpy.node import Node
from dashboard.backend.base_node import BaseDashboardNode
from std_msgs.msg import String, Float64MultiArray, Float64, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class MobilityNode(BaseDashboardNode):
    """
    Mobility dashboard ROS 2 node.
    Default node name: "karura_mobility_gui"
    """

    def __init__(self, node_name: str = "karura_mobility_gui"):
        super().__init__(node_name)

        # mobility node subscribers
        self.mobility_status_sub = self.create_subscription(String, "mobility_status", self.mobility_status_callback, 10)
        self.actual_rads_sub = self.create_subscription(Float64MultiArray, "actual_rads", self.actual_rads_callback, 10)
        self.actual_angle_sub = self.create_subscription(Float64MultiArray, "actual_angle", self.actual_angle_callback, 10)
        self.rpy_sub = self.create_subscription(Float64MultiArray, "roll_pitch_yaw", self.rpy_callback, 10)
        self.odometry_sub = self.create_subscription(Odometry, "odometry_filtered", self.odometry_callback, 10)
        self.bandwidth_sub = self.create_subscription(Float64, "bandwidth", self.bandwidth_callback, 10)
        self.battery_data_sub = self.create_subscription(Int32, "battery_data", self.battery_data_callback, 10)
        self.navigation_status_sub = self.create_subscription(String, "navigation_status", self.navigation_status_callback, 10)
        self.fisheye_cam_sub = self.create_subscription(String, "fisheye_cam", self.fisheye_cam_callback, 10)
        self.gps_map_sub = self.create_subscription(Float64MultiArray, "gps_map", self.gps_map_callback, 10)
        self.gps_data_sub = self.create_subscription(NavSatFix, "gps_data", self.gps_data_callback, 10)
        self.battery_voltage_data_sub = self.create_subscription(Float64, "temperature_data", self.battery_voltage_data_callback, 10)
        self.battery_power_data_sub = self.create_subscription(Float64, "humidity_data", self.battery_power_data_callback, 10)
        self.get_logger().info("[MobilityNode] Telemetry subscribers initialized.")

        # publisher for cmd_vel
    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

        self.get_logger().info(f"[MobilityNode] PUBLISHED cmd_vel → linear={linear_x:.2f}, angular={angular_z:.2f}")

    # callbacks
    def mobility_status_callback(self, msg: String):
        try:
            self.get_logger().info(f"Received status: {msg.data}")
            # Try dispatching the object first
            self._dispatch("mobility_status", msg)
        except Exception as e:
            self.get_logger().error(f"Dispatch error: {e}")

    def actual_rads_callback(self, msg: Float64MultiArray):
        try:
            self.get_logger().info(f"Received rads: {len(msg.data)} values")
            self._dispatch("actual_rads", msg)
        except Exception as e:
            self.get_logger().error(f"Dispatch error: {e}")

    def actual_angle_callback(self, msg: Float64MultiArray):
        try:
            self.get_logger().info(f"Received angles: {len(msg.data)} values")
            self._dispatch("actual_angle", msg)
        except Exception as e:
            self.get_logger().error(f"Dispatch error: {e}")

    def rpy_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f"roll_pitch_yaw: {msg.data}")
        self._dispatch("roll_pitch_yaw", msg.data)

    def odometry_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z

        self.get_logger().info(f"odometry_filtered: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self._dispatch("odometry_filtered", {"x": x, "y": y, "z": z})

    def bandwidth_callback(self, msg: Float64):
        self.get_logger().info(f"bandwidth: {msg.data}")
        self._dispatch("bandwidth", msg.data)

    def battery_data_callback(self, msg: Int32):
        self.get_logger().info(f"battery_data: {msg.data}")
        self._dispatch("battery_data", msg.data)

    def battery_voltage_data_callback(self, msg: Float64):
            self.get_logger().info(f"battery_voltage_data: {msg.data}")
            self._dispatch("battery_voltage_data", msg.data)

    def battery_power_data_callback(self, msg: Float64):
        self.get_logger().info(f"battery_power_data: {msg.data}")
        self._dispatch("battery_power_data", msg.data)

    def navigation_status_callback(self, msg: String):
        self.get_logger().info(f"navigation_status: {msg.data}")
        self._dispatch("navigation_status", msg.data)

    def fisheye_cam_callback(self, msg: String):
        self.get_logger().info(f"fisheye_cam: {msg.data}")
        self._dispatch("fisheye_cam", msg.data)

    def gps_map_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f"gps_map: {msg.data}")
        self._dispatch("gps_map", msg.data)

    def gps_data_callback(self, msg: NavSatFix):
        data = {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude
        }

        self.get_logger().info(
            f"GPS data received: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}"
        )

        self._dispatch("gps_data", data)


def main():
    rclpy.init(args=sys.argv)
    node = MobilityNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  
    main()
