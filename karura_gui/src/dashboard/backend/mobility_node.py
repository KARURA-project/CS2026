"""
MobilityNode

ROS 2 node responsible for mobility-related topics for the Karura dashboard GUI.

TODO: fill in publishers/subscribers for mobility-only topics
using the BaseDashboardNode._dispatch() pattern.
"""
import sys
from .base_node import BaseDashboardNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

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
        self.i = 0
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("Mobility node ready (Twist publisher + sububscriber).")

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
        temp_linear = 0 # temp variables
        temp_angular = 0
        msg = Twist()
        msg.linear.x = temp_linear
        msg.angular.z = temp_angular
        self.cmd_vel_pub.publish(msg)

        self.get_logger().info(f"[MobilityNode] Publisher, cmd_vel linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
        self.i += 1

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear
        angular = msg.angular
        self.get_logger().info(
            f"[MobilityNode] Subscriber, cmd_vel received, linear: ({linear.x:2f}, {linear.y:.2f}, {linear.z:.2f}),"
            f" angular: ({angular.x:.2f}, {angular.y:.2f}, {angular.z:.2f})"
        )


def main():
    rclpy.init(args=sys.argv)
    node = MobilityNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
