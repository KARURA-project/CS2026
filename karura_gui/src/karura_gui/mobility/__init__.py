"""
This is following the format from the science GUI. The explanation is that we are
exporting the mobility UI from the dashboard package so callers from karura_gui.mobility
can import MobilityMainWindow and MobilityBridge while the real implementation lives
under dashboard.mobility
"""
from dashboard.mobility.window import MobilityMainWindow
from dashboard.mobility.bridge import MobilityBridge #TODO
__all__ = ["MobilityMainWindow", "MobilityBridge"]