import rclpy
from PySide6.QtCore import Signal

from dashboard.backend.arm_node import ArmNode
from dashboard.backend.base_bridge import BaseROS2Bridge


from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState, Image

class ArmBridge(BaseROS2Bridge):
    delta_twist_cmds_signal = Signal(TwistStamped)
    delta_joint_cmds_signal = Signal(JointJog)
    hand_status_signal = Signal(Float64MultiArray)
    joint_states_signal = Signal(JointState)
    arm_targets_signal = Signal(Float64MultiArray)
    arm_current_signal = Signal(Float64MultiArray)
    hand_cam_1_signal = Signal(Image)
    hand_cam_2_signal = Signal(Image)
    
    error_signal = Signal(str)
    
    def __init__(self, arm_node_name: str = "karura_arm_gui"):
        
        
        if not rclpy.ok():  
            rclpy.init(args=None)
            
        self.arm_node = ArmNode(arm_node_name)
        
        super().__init__([self.arm_node])
        self.ros_error.connect(self._on_ros_error)
        
        self.arm_node.register_callback("delta_twist_cmds", self._emit_delta_twist_cmds)
        self.arm_node.register_callback("delta_joint_cmds", self._emit_delta_joint_cmds)
        self.arm_node.register_callback("hand_status", self._emit_hand_status)
        self.arm_node.register_callback("joint_states", self._emit_joint_states)
        self.arm_node.register_callback("arm_targets", self._emit_arm_targets)
        self.arm_node.register_callback("arm_current", self._emit_arm_current)
        self.arm_node.register_callback("hand_cam_1", self._emit_hand_cam_1)
        self.arm_node.register_callback("hand_cam_2", self._emit_hand_cam_2)
                                        
        self.ros_error.connect(self._on_ros_error)
        
    # emitters
    def _emit_delta_twist_cmds(self, msg: TwistStamped):
        try:
            self.arm_status_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting delta_twist_cmds: {e}")
    
    def _emit_delta_joint_cmds(self, msg: JointJog):
        try:
            self.delta_joint_cmds_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting delta_joint_cmds: {e}")
            
    def _emit_hand_status(self, msg: Float64MultiArray):
        try:
            self.hand_status_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting hand_status: {e}")
            
    def _emit_joint_states(self, msg: JointState):
        try:
            self.joint_states_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting joint_states: {e}")
            
    def _emit_arm_targets(self, msg: Float64MultiArray):
        try:
            self.arm_targets_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting arm_targets: {e}")
            
    def _emit_arm_current(self, msg: Float64MultiArray):
        try:
            self.arm_current_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting arm_current: {e}")      
    
    def _emit_hand_cam_1(self, msg: Image):
        try:
            self.hand_cam_1_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting hand_cam_1: {e}")
            
    def _emit_hand_cam_2(self, msg: Image):
        try:
            self.hand_cam_2_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting hand_cam_2: {e}")   
            
    def _on_ros_error(self, error_msg: str):
        self.error_signal.emit(f"ROS 2 Error: {error_msg}")