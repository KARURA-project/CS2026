# dashboard/core/base_bridge.py
from typing import Iterable
import rclpy
from PySide6.QtCore import QObject, Signal

from .ros2_worker import ROS2Worker


class BaseROS2Bridge(QObject):
    ros_error = Signal(str)

    def __init__(self, nodes: Iterable):
        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.nodes = list(nodes)

        self.worker = ROS2Worker(self.nodes, parent=self)
        self.worker.error.connect(self.ros_error)

    def start(self):
        self.worker.start()

    def shutdown(self):
        self.worker.stop()

        for n in self.nodes:
            try:
                n.destroy_node()
            except Exception:
                pass

        try:
            rclpy.shutdown()
        except Exception:
            pass
