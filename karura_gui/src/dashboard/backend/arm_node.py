"""
ArmNode

ROS 2 node responsible for arm-related topics for the Karura dashboard GUI.

TODO: fill in publishers/subscribers for arm-only topics
using the BaseDashboardNode._dispatch() pattern.
"""

from .base_node import BaseDashboardNode


class ArmNode(BaseDashboardNode):
    """
    Arm dashboard ROS 2 node.
    Default node name: "karura_arm_gui"
    """

    def __init__(self, node_name: str = "karura_arm_gui"):
        super().__init__(node_name)
