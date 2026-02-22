from PySide6.QtCore import QObject, Signal

class MobilityBridge(QObject):
    actual_rads_signal = Signal(list)
    rpy_signal = Signal(float, float, float)
