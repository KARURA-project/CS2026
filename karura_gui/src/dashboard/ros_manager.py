import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

class ROSManager:

    def __init__(self, node_classes: list, gui_callbacks: dict):
        """
        node_classes: list of ROS2 Node classes to instantiate
            Example:
                [MobilityNode, ArmNode, SensorNode]

        gui_callbacks: dict mapping:
            {
                "MobilityNode": {
                    "actual_rads": gui_fn,
                    "cmd_vel": gui_fn,
                },
                "ArmNode": {
                    "arm_position": gui_fn,
                },
                "SensorNode": {
                    "lidar_scan": gui_fn,
                }
            }
        """

        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.nodes = []

        # Instantiate each node
        for node_class in node_classes:
            node = node_class()
            self.nodes.append(node)
            self.executor.add_node(node)

            # Register GUI callbacks if provided
            node_name = node.__class__.__name__
            if node_name in gui_callbacks:
                for topic_name, callback in gui_callbacks[node_name].items():
                    node.register_callback(topic_name, callback)

        # Spin ROS2 in a background thread
        self.thread = threading.Thread(target=self._run_executor, daemon=True)

    def start(self):
        self.thread.start()

    def _run_executor(self):
        self.executor.spin()

    def get_node(self, node_class):
        """Return the node instance so GUI can call publishers."""
        for n in self.nodes:
            if isinstance(n, node_class):
                return n
        return None

    def shutdown(self):
        self.executor.shutdown()
        for n in self.nodes:
            n.destroy_node()
        rclpy.shutdown()
