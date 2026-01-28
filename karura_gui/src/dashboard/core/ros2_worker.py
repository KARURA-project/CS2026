# core/ros2_worker.py
import rclpy
from rclpy.executors import MultiThreadedExecutor
from PySide6.QtCore import Signal, QThread


class ROS2Worker(QThread):
    error = Signal(str)

    def __init__(self, nodes, parent=None):
        """
        nodes: list[rclpy.node.Node]
        """
        super().__init__(parent)
        self.nodes = list(nodes)
        self._running = True
        self.executor = MultiThreadedExecutor()

        for n in self.nodes:
            self.executor.add_node(n)

    def run(self):
        try:
            while self._running and rclpy.ok():
                # spin callbacks/timers for ALL nodes added to executor
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._running = False

        # Safely detach nodes from executor (helps clean shutdown)
        try:
            for n in self.nodes:
                self.executor.remove_node(n)
        except Exception:
            pass

        self.quit()
        self.wait()
