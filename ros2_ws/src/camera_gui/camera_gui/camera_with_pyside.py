import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPixmap, QImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiCameraGUI(QMainWindow):
    def __init__(self, num_cameras = 5):
        super().__init__()
        self.setWindowTitle("Multi-Camera Viewer")
        self.bridge = CvBridge()
        self.num_cameras = num_cameras
        self.frames = [np.zeros((240, 320, 3), dtype=np.uint8) for _ in range(num_cameras)]

        #This is the delay per loop
        self.time_delay = 10
        self.display_delay = 33

        #Set up ROS2
        if not rclpy.ok():
            rclpy.init(args = None)

        self.ros_node = Node('pyside6_multi_camera_node')

        for i in range(num_cameras):
            topic = f'/camera{i}/image_raw'
            self.ros_node.create_subscription(
                Image, topic, self.make_callback(i), self.time_delay
            )
        
        #Creates single QLabel to display combined OpenCV image
        self.image_label = QLabel(self)
        self.setCentralWidget(self.image_label)

        #Sets a timer for ROS 2 to receive data 
        self.ros_spin_timer = QTimer(self)
        self.ros_spin_timer.timeout.connect(self.spin_ros)
        self.ros_spin_timer.start(self.time_delay)

        #Sets a timer to update the display
        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self.update_display)
        self.ros_spin_timer.start(self.display_delay)    

    #Processes ROS 2 callbacks
    def spin_ros(self):
        if rclpy.ok(): #Makes sure context is running when looping. Needed in order for interrupt signal (SIGINT) to work properly
            rclpy.spin_once(self.ros_node, timeout_sec = 0)

    #Saves frame
    def make_callback(self, index):
        def callback(msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.frames[index] = frame
            except Exception as e:
                self.ros_node.get_logger().error(f"CV Bridge Error: {e}")
        return callback
    
    #Combines 
    def update_display(self):
        #Combines OpenCV frames and displays using PySide6
        try:
            # Recreate your OpenCV combination logic (display_all from previous code)
            resized = [cv2.resize(f, (320, 240)) for f in self.frames]
            top = cv2.hconcat(resized[0:3])
            
            # Handle the bottom row padding as before
            bottom_row = resized[3:5]
            if len(bottom_row) < 3:
                # Use np.zeros_like to create a black patch matching the size
                black_patch = np.zeros_like(resized[0])
                bottom_row += [black_patch] * (3 - len(bottom_row))
                
            bottom = cv2.hconcat(bottom_row)
            combined_bgr = cv2.vconcat([top, bottom])
            
            # --- 4. Conversion to QPixmap ---
            height, width, channel = combined_bgr.shape
            bytes_per_line = 3 * width
            
            # Convert BGR (OpenCV) to RGB (Qt standard)
            combined_rgb = cv2.cvtColor(combined_bgr, cv2.COLOR_BGR2RGB) 
            
            q_image = QImage(
                combined_rgb.data, 
                width, 
                height, 
                bytes_per_line, 
                QImage.Format.Format_RGB888
            )
            
            pixmap = QPixmap.fromImage(q_image)
            self.image_label.setPixmap(pixmap)
            self.image_label.adjustSize()
            
        except Exception as e:
            self.ros_node.get_logger().warn(f"Display update error: {e}")

    def closeEvent(self, event):
        #Cleans up ROS 2 resources when GUI closes
        self.ros_spin_timer.stop()
        self.display_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = MultiCameraGUI(num_cameras=5)
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()