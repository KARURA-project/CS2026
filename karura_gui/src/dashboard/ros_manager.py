import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

# ai generated ros_manager.
class ROSManager:

    def __init__(self, node_classes: list, gui_callbacks: dict):

        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.nodes = []

        # Instantiate and register nodes
        for node_class in node_classes:
            node = node_class()
            self.nodes.append(node)
            self.executor.add_node(node)

            node_name = node.__class__.__name__

            # Register GUI callbacks if provided
            if node_name in gui_callbacks:
                for topic_name, callback in gui_callbacks[node_name].items():

                    # Only register the callback if the node supports it
                    if hasattr(node, "register_callback"):
                        node.register_callback(topic_name, callback)
                    else:
                        node.get_logger().warn(
                            f"Node {node_name} does not support register_callback(), "
                            f"skipping callback for topic '{topic_name}'."
                        )

        # Background ROS thread
        self.thread = threading.Thread(
            target=self._run_executor, daemon=True
        )

    def start(self):
        """Start the ROS executor thread."""
        self.thread.start()

    def _run_executor(self):
        """Spin executor in a separate thread."""
        self.executor.spin()

    def get_node(self, node_class):
        """
        Retrieve the node instance so GUI can call publisher functions.
        Example:
            mobility = ros_manager.get_node(MobilityNode)
            mobility.publish_cmd_vel(0.2, 0.0)
        """
        for n in self.nodes:
            if isinstance(n, node_class):
                return n
        return None

    def shutdown(self):
        """Gracefully stop ROS2."""
        self.executor.shutdown()
        for n in self.nodes:
            n.destroy_node()
        rclpy.shutdown()
