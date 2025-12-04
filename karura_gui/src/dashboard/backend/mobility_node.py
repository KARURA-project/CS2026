"""
MobilityNode

ROS 2 node responsible for mobility-related topics for the Karura dashboard GUI.

TODO: fill in publishers/subscribers for mobility-only topics
using the BaseDashboardNode._dispatch() pattern.
"""
import sys
from base_node import BaseDashboardNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class MobilityNode(BaseDashboardNode):
    """
    Mobility dashboard ROS 2 node.
    Default node name: "karura_mobility_gui"
    """

    def __init__(self, node_name: str = "karura_mobility_gui"):
        super().__init__(node_name)

        # mobility node publisher/subscriber
        self.cmd_publisher = self.create_publisher(String, 'mobility_cmd', 10)
        self.status_subscriber = self.create_subscription(String, 'mobility_status', self.status_callback, 10)
        self.get_logger().info("MobilityNode initialized (publisher + subscriber).")

        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.counter = 0

        # cmd_vel publisher/subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velocity_timer = self.create_timer(1.0, self.publish_velocity)
        self.vel_counter = 0
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("[Mobility] cmd_vel node ready (Twist publisher + sububscriber).")

        # actual_rads publisher/subscriber
        self.actual_rads_pub = self.create_publisher(Float64MultiArray, 'actual_rads', 10)
        self.actual_rads_timer = self.create_timer(1.0, self.publish_actual_rads)
        self.rad_counter = 0
        self.actual_rads_sub = self.create_subscription(Float64MultiArray, 'actual_rads', self.actual_rads_callback, 10)
        self.get_logger().info("[Mobility] actual_rads node ready (Float64MultiArray publisher + subscriber).")

        # roll_pitch_yaw publisher/subscriber (rpy = roll_pitch_yaw)
        self.rpy_pub = self.create_publisher(Vector3, 'roll_pitch_yaw', 10)
        self.rpy_timer = self.create_timer(1.0, self.publish_rpy)
        self.rpy_counter = 0
        self.rpy_sub = self.create_subscription(Vector3, 'roll_pitch_yaw', self.rpy_callback, 10)
        self.get_logger().info("[Mobility] roll_pitch_yaw node ready (Vector3 publisher + subscriber).")

        # odometry_filtered publisher/subscriber
        self.odometry_pub = self.create_publisher(Odometry, 'odometry_filtered', 10)
        self.odometry_timer = self.create_timer(1.0, self.publish_odometry)
        self.odometry_counter = 0
        self.odometry_sub = self.create_subscription(Odometry, 'odometry_filtered', self.odometry_callback, 10)
        self.get_logger().info("[Mobility] odometry_filtered node ready (Odometry publisher + subscriber)")

    # We have a timer here to send a "heartbeat" every 10 seconds to check on the node.
    def publish_heartbeat(self):
        msg = String()
        msg.data = f"GUI heartbeat {self.counter}"
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"[MobilityNode] Publishing: {msg.data}")
        self.counter += 1

    def status_callback(self, msg: String):
        self.get_logger().info(f"[MobilityNode] Received status: {msg.data}")

    def publish_velocity(self):
        temp_linear = 0  # TODO, fill in temp variables
        temp_angular = 0
        msg = Twist()
        msg.linear.x = temp_linear
        msg.angular.z = temp_angular
        self.cmd_vel_pub.publish(msg)

        self.get_logger().info(f"[MobilityNode] Publisher, cmd_vel linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
        self.vel_counter += 1

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear
        angular = msg.angular
        self.get_logger().info(
            f"[MobilityNode] Subscriber, cmd_vel received, linear: ({linear.x:.2f}, {linear.y:.2f}, {linear.z:.2f}),"
            f" angular: ({angular.x:.2f}, {angular.y:.2f}, {angular.z:.2f})"
        )

    def publish_actual_rads(self):
        msg = Float64MultiArray()

        # TODO fill in temporary variables (0s and 1s)
        msg.data = [
            0 + self.rad_counter * 1,
            0 + self.rad_counter * 1,
            0 + self.rad_counter * 1,
            0 + self.rad_counter * 1
        ]
        self.actual_rads_pub.publish(msg)
        self.get_logger().info(f"[MobilityNode] Publisher, actual_rads value: {msg.data}")
        self.rad_counter += 1

    def actual_rads_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f"[MobilityNode] Subscriber, actual_rads received: {msg.data}")

    def publish_rpy(self):
        msg = Vector3()

        # TODO fill in temporary variables (0.1, 0.05, 0.2)
        msg.x = 0.1 * self.rpy_counter # roll
        msg.y = 0.05 * self.rpy_counter # pitch
        msg.z = 0.2 * self.rpy_counter # yaw
        self.rpy_pub.publish(msg)
        self.get_logger().info(f"[MobilityNode] Publisher, roll_pitch_yaw values: roll={msg.x:.2f} pitch={msg.y:.2f} yaw={msg.z:.2f}")
        self.rpy_counter += 1

    def rpy_callback(self, msg: Vector3):
        self.get_logger().info(f"[MobilityNode] Subscriber, roll_pitch_yaw values received: "
                              f"roll={msg.x:.2f} pitch={msg.y:.2f} yaw={msg.z:.2f}")

    def publish_odometry(self):
        msg = Odometry()
        msg.header.frame_id = "odometry_filtered"
        msg.child_frame_id = "base_link"

        # TODO fill in temp variables for pose and velocity
        msg.pose.pose.position.x = 0.1 * self.odometry_counter
        msg.pose.pose.position.y = 0
        msg.pose.pose.position.z = 0

        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = 0.1
        msg.twist.twist.angular.z = 0.01

        self.odometry_pub.publish(msg)
        self.get_logger().info(f"[MobilityNode] Publisher, odometry_filtered values: x={msg.pose.pose.position.x:.2f},"
                               f" vx={msg.twist.twist.linear.x:.2f}")
        self.odometry_counter += 1

    def odometry_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta_z = msg.pose.pose.orientation.z

        self.get_logger().info(f"[MobilityNode] Subscriber, odometry_filtered received: "
                               f"x={x:.2f} y={y:.2f} z={theta_z:.2f}")


def main():
    rclpy.init(args=sys.argv)
    node = MobilityNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
