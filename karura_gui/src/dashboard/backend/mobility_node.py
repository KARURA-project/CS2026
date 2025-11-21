"""
MobilityNode

ROS 2 node responsible for mobility-related topics for the Karura dashboard GUI.

TODO: fill in publishers/subscribers for mobility-only topics
using the BaseDashboardNode._dispatch() pattern.
"""

from .base_node import BaseDashboardNode


class MobilityNode(BaseDashboardNode):
    """
    Mobility dashboard ROS 2 node.
    Default node name: "karura_mobility_gui"
    """

    def __init__(self, node_name: str = "karura_mobility_gui"):
        super().__init__(node_name)