import rclpy
from rclpy.node import Node
import random
import math

from std_msgs.msg import String, Float64MultiArray, Float64, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class MobilitySimProvider(Node):
    def __init__(self):
        super().__init__('mobility_sim_provider')
        
        # Publishers matching MobilityNode subscriptions
        self.pub_status = self.create_publisher(String, "mobility_status", 10)
        self.pub_rads = self.create_publisher(Float64MultiArray, "actual_rads", 10) #up down stop up down. rotation keep at 0
        self.pub_rpy = self.create_publisher(Float64MultiArray, "roll_pitch_yaw", 10)
        self.pub_odom = self.create_publisher(Odometry, "odometry_filtered", 10)
        self.pub_bandwidth = self.create_publisher(Float64, "bandwidth", 10)
        self.pub_battery = self.create_publisher(Int32, "battery_data", 10)
        self.pub_battery_voltage = self.create_publisher(Float64, "battery_voltage_data", 10)   
        self.pub_battery_power = self.create_publisher(Float64, "battery_power_data", 10)
        self.pub_nav_status = self.create_publisher(String, "navigation_status", 10)
        self.pub_gps_data = self.create_publisher(NavSatFix, "gps_data", 10)
        self.pub_angle = self.create_publisher(Float64MultiArray, "actual_angle", 10)

        

        # Timer to publish at 2Hz (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.publish_random_data)
        self.get_logger().info("Mobility Simulation Provider Started...")
        
        # Internal state for smooth odom simulation
        self.step = 0.0

    def publish_random_data(self):
        self.step += 0.1
        
        # 1. Mobility Status (Cycling strings)
        status_msg = String()
        status_msg.data = random.choice(["READY", "MOVING", "AUTONOMOUS", "MANUAL"])
        self.pub_status.publish(status_msg)

        # 2. GPS Data (Simulating a slight walk around a coordinate)
        gps_msg = NavSatFix()
        gps_msg.latitude = 37.7749 + (random.uniform(-0.001, 0.001))
        gps_msg.longitude = -122.4194 + (random.uniform(-0.001, 0.001))
        gps_msg.altitude = 15.0 + random.uniform(-0.5, 0.5)
        self.pub_gps_data.publish(gps_msg)

        # 3. Actual Rads and Angles (4 wheels/motors)
        #Use step to create a seed for consistent "random" behavior per 60s cycle
        rng = random.Random(int(self.step // 60))
        t_start, v_max = rng.uniform(2, 10), rng.uniform(2, 8)
        t_mid, t_end = rng.uniform(20, 30), rng.uniform(45, 55)

        # Velocity Profile: Speed up, Slow down, Stop, Speed up, Slow down
        # A simple absolute sine of the clock creates the "two-hump" speed profile
        clock = (self.step * 10) % 60
        active = t_start < clock < t_end
        # Multi-wave sine creates realistic acceleration/deceleration humps
        target_speed = v_max * abs(math.sin((clock - t_start) * 0.15)) if active else 0.0

        # Publish Rads
        rads_msg = Float64MultiArray()
        rads_msg.data = [max(0.0, target_speed + random.uniform(-0.1, 0.1)) if active else 0.0 for _ in range(4)]
        self.pub_rads.publish(rads_msg)

        # Battery Power (Simplified: Speed * Voltage / 10)
        batt_pow_msg = Float64()
        batt_pow_msg.data = 24.0
        self.pub_battery_power.publish(batt_pow_msg)

        angle_msg = Float64MultiArray()
        angle_msg.data = [0.0, 0.0, 0.0, 0.0] 
        self.pub_angle.publish(angle_msg)

        # 4. Battery Data 
        batt_msg = Int32()
        batt_msg.data = int(92 - (self.step / 69 + .1))
        self.pub_battery.publish(batt_msg)
        
        # 5. Corrected Battery Voltage
        batt_volt_msg = Float64()
        batt_volt_msg.data = 21 + (random.random() * 0.02) + (sum(rads_msg.data) * 0.012)
        self.pub_battery_voltage.publish(batt_volt_msg)

        # 6. Corrected Battery Power
        batt_pow_msg = Float64()
        batt_pow_msg.data = 24.0   
        self.pub_battery_power.publish(batt_pow_msg) 
                               
        # 7. Odometry (Simulating a circle)
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = 5.0 * math.cos(self.step)
        odom_msg.pose.pose.position.y = 5.0 * math.sin(self.step)
        odom_msg.pose.pose.orientation.z = math.sin(self.step / 2) # simplified yaw
        self.pub_odom.publish(odom_msg)

        # 8. Bandwidth (Mbps)
        bw_msg = Float64()
        bw_msg.data = random.uniform(15.5, 45.2)
        self.pub_bandwidth.publish(bw_msg)

        self.get_logger().info("Publish round finished")

def main(args=None):
    rclpy.init(args=args)
    node = MobilitySimProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()