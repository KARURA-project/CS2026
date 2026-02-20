"""
ArmNode

ROS 2 node responsible for arm-related topics for the Karura dashboard GUI.

TODO: fill in publishers/subscribers for arm-only topics
using the BaseDashboardNode._dispatch() pattern.
"""

import sys
import rclpy
from rclpy.node import Node
from dashboard.backend.base_node import BaseDashboardNode
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState, Image


from .base_node import BaseDashboardNode


class ArmNode(BaseDashboardNode):
    """
    Arm dashboard ROS 2 node.
    Default node name: "karura_arm_gui"
    """

    def __init__(self, node_name: str = "karura_arm_gui"):
        super().__init__(node_name)
        
        # arm node subscribers
        self.delta_twist_cmds_sub = self.create_subscription(TwistStamped, "delta_twist_cmds", self.delta_twist_cmds_callback, 10)
        self.delta_joint_cmds_sub = self.create_subscription(JointJog, "delta_joint_cmds", self.delta_joint_cmds_callback, 10)
        self.hand_status_sub = self.create_subscription(Float64MultiArray, "hand_status", self.hand_status_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState, "joint_states", self.joint_states_callback, 10)
        self.arm_targets_sub = self.create_subscription(Float64MultiArray, "arm_targets", self.arm_targets_callback, 10)
        self.arm_current_sub = self.create_subscription(Float64MultiArray, "arm_current", self.arm_current_callback, 10)
        self.hand_cam_1_sub = self.create_subscription(Image, "hand_cam_1", self.hand_cam_1_callback, 10)
        self.hand_cam_2_sub = self.create_subscription(Image, "hand_cam_2", self.hand_cam_2_callback, 10)
        
        self.get_logger().info("[ArmNode] subscribers initialized")
        
        
    def delta_twist_cmds_callback(self, msg: TwistStamped):
        self.get_loger().info(f"delta_twist_cmds: {msg.data}")
        self._dispatch("delta_twist_cmds", msg.data)

    def delta_joint_cmds_callback(self, msg: JointJog):
        self.get_loger().info(f"delta_joint_cmds: {msg.data}")
        self._dispatch("delta_joint_cmds", msg.data)

    def hand_status_callback(self, msg: Float64MultiArray):
        self.get_loger().info(f"hand_status: {msg.data}")
        self._dispatch("hand_status", msg.data)
        
    def joint_states_callback(self, msg: JointState):
        self.get_loger().info(f"joint_states: {msg.data}")
        self._dispatch("joint_states", msg.data)
        
    def arm_targets_callback(self, msg: Float64MultiArray):
        self.get_loger().info(f"arm_targets: {msg.data}")
        self._dispatch("arm_targets", msg.data)
        
    def arm_current_callback(self, msg: Float64MultiArray):
        self.get_loger().info(f"arm_current: {msg.data}")
        self._dispatch("arm_current", msg.data)

    def hand_cam_1_callback(self, msg: Image):
        self.get_loger().info(f"hand_cam_1: {msg.data}")
        self._dispatch("hand_cam_1", msg.data)
        
    def hand_cam_2_callback(self, msg: Image):
        self.get_loger().info(f"hand_cam_2: {msg.data}")
        self._dispatch("hand_cam_2", msg.data)

def main():
    rclpy.init(args=sys.argv)
    node = ArmNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()