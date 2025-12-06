
from .base_node import BaseDashboardNode

class CommsNode(BaseDashboardNode):
    """
    Communications dashboard ROS 2 node.
    Default node name: "karura_comms_gui"
    """

    def __init__(self, node_name: str = "karura_comms_gui"):
        super().__init__(node_name)
        
        