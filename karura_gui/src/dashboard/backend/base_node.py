import rclpy
from rclpy.node import Node

class BaseDashboardNode(Node):
    """
    _summary_
    This is the base class responsible for all the dashboard nodes.
    - Store a mapping of callback functions for different topics.
    - Provide a method to register callbacks for specific topics.
    
    To implement a topic subscription, inherit from this class and use 
    the register_callback method to associate a topic name with a callback function.
    
    __Usage pattern in subclasses__:
    
        self.create_subscription(
            SomeMsgType,
            "/some/topic",
            self._on_some_topic,
            10,
        )

        def _on_some_topic(self, msg: SomeMsgType):
            self._dispatch("some_topic", msg)

    The GUI-side bridge will then do:
        node.register_callback("some_topic", self._emit_some_topic)
    
    __Args__:
        Node (_type_): _description_
    """
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._callbacks = {}  # key -> list[callback]

    def register_callback(self, name: str, callback):
        self._callbacks.setdefault(name, []).append(callback)

    def _dispatch(self, key: str, msg):
        for cb in self._callbacks.get(key, []):
            cb(msg)