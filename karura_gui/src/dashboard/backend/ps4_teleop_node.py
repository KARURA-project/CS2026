from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from dashboard.backend.base_node import BaseDashboardNode


class PS4TeleopNode(BaseDashboardNode):
    def __init__(self, node_name: str = "karura_ps4_teleop"):
        super().__init__(node_name)

        self.max_linear_speed = 0.5
        self.max_angular_speed = 2.0
        self.deadzone = 0.15

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.enabled = True
        self.estop = False

        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)

        self.get_logger().info("[PS4TeleopNode] Sub /joy, Pub /cmd_vel")

    def apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def joy_callback(self, msg: Joy):
        axes = msg.axes

        left_y = self.apply_deadzone(axes[1]) if len(axes) > 1 else 0.0
        l2 = (1.0 - axes[2]) / 2.0 if len(axes) > 2 else 0.0
        r2 = (1.0 - axes[5]) / 2.0 if len(axes) > 5 else 0.0

        self.target_linear = -left_y * self.max_linear_speed
        self.target_angular = (r2 - l2) * self.max_angular_speed

        if abs(self.target_linear) < 0.01:
            self.target_linear = 0.0
        if abs(self.target_angular) < 0.01:
            self.target_angular = 0.0

        # Optional: let GUI display current command
        self._dispatch("teleop_cmd", {"linear": self.target_linear, "angular": self.target_angular})

    def publish_cmd_vel(self):
        if not self.enabled or self.estop:
            self.cmd_vel_pub.publish(Twist())
            return

        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.cmd_vel_pub.publish(msg)

    # GUI-callable hooks
    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)
        if not self.enabled:
            self.cmd_vel_pub.publish(Twist())

    def trigger_estop(self):
        self.estop = True
        self.cmd_vel_pub.publish(Twist())

    def clear_estop(self):
        self.estop = False
