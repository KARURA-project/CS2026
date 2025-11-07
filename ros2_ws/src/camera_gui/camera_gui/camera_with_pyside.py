import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QGridLayout, QWidget
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPixmap, QImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Define the number of local cameras to open
NUM_LOCAL_CAMERAS = 1 # Example: You might use 0 and 1
# Define the size for local cameras to match the ROS feed/display size
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

class MultiCameraGUI(QMainWindow):
    def __init__(self, num_ros_cameras = 5, num_local_cameras = NUM_LOCAL_CAMERAS):
        super().__init__()
        self.setWindowTitle("Multi-Camera Viewer (ROS & Local)")
        self.bridge = CvBridge()
        
        # ROS 2 Setup
        self.num_ros_cameras = num_ros_cameras

        # Stores frames from ROS topics
        self.ros_frames = [np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) for _ in range(num_ros_cameras)]
        self.num_local_cameras = num_local_cameras

        # Stores frames from local webcams
        self.local_frames = [np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) for _ in range(num_local_cameras)]
        self.captures = []
        
        print("--- Initializing Local Cameras ---")
        for i in range(num_local_cameras):
            # Attempt to open the camera using its device index (0, 1, 2, ...)
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            
            if not cap.isOpened():
                print(f"Warning: Could not open local camera at index {i}. Appending None.")
                self.captures.append(None)
                continue
            
            # Set properties
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            
            self.captures.append(cap)
            print(f"Successfully initialized local camera {i} at {FRAME_WIDTH}x{FRAME_HEIGHT}.")
        
        # Combine ROS and Local frames into one list for display logic
        self.all_frames = self.ros_frames + self.local_frames
        
        # Timing (in milliseconds)
        self.ros_spin_delay = 10   # 100Hz for ROS callbacks
        self.display_delay = 33    # ~30Hz for display updates and camera reading

        # Set up ROS 2
        if not rclpy.ok():
            rclpy.init(args = None)

        self.ros_node = Node('pyside6_multi_camera_node')

        for i in range(num_ros_cameras):
            topic = f'/camera{i}/image_raw'
            # Note: The 'index' here refers to the position in the ros_frames list
            self.ros_node.create_subscription(
                Image, topic, self.make_ros_callback(i), 10 # QoS depth
            )
        
        # Creates single QLabel to display combined OpenCV image
        self.image_label = QLabel(self)
        self.setCentralWidget(self.image_label)

        # Timer for ROS 2 (Spinning the node)
        self.ros_spin_timer = QTimer(self)
        self.ros_spin_timer.timeout.connect(self.spin_ros)
        self.ros_spin_timer.start(self.ros_spin_delay)

        # Timer for updating the display AND reading local cameras
        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self.read_local_cameras_and_update_display)
        self.display_timer.start(self.display_delay) 

    # Processes ROS 2 callbacks (unchanged)
    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec = 0)

    # ROS 2 Callback function factory (saves frame to ros_frames list)
    def make_ros_callback(self, index):
        def callback(msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.ros_frames[index] = frame
            except Exception as e:
                self.ros_node.get_logger().error(f"ROS Bridge Error: {e}")
        return callback

    def read_local_cameras_and_update_display(self):
        # 1. Read Local Camera Frames
        for i, cap in enumerate(self.captures):
            if cap is not None:
                ret, frame = cap.read()
                if ret:
                    # Update the local frame buffer
                    self.local_frames[i] = frame
                # Note: No need for else/error handling here unless you want 
                # to continuously check for disconnections.
        
        # 2. Update the combined frame list
        self.all_frames = self.ros_frames + self.local_frames
        
        # 3. Call display method
        self.update_display()
        
    # Combines frames and displays using PySide6 (now uses self.all_frames)
    def update_display(self):
        try:
            # Recreates OpenCV combination logic (display_all from previous code)
            # Use the combined list of all frames (ROS + Local)
            resized = [cv2.resize(f, (FRAME_WIDTH, FRAME_HEIGHT)) for f in self.all_frames]
            
            # The rest of the tiling logic needs to account for the total number of frames
            total_frames = len(resized)
            
            # Assuming 3 frames per row for tiling (total of 5 ROS + 2 Local = 7 frames)
            row_size = 3 
            
            # Divide into rows
            rows = []
            for i in range(0, total_frames, row_size):
                current_row = resized[i : i + row_size]
                # Pad the last row with black frames if necessary
                if len(current_row) < row_size:
                    black_patch = np.zeros_like(resized[0])
                    current_row += [black_patch] * (row_size - len(current_row))
                
                rows.append(cv2.hconcat(current_row))
            
            # Vertically concatenate all rows
            if not rows:
                combined_bgr = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) # Default black if no frames
            else:
                combined_bgr = cv2.vconcat(rows)
            
            # --- Conversion to QPixmap ---
            height, width, channel = combined_bgr.shape
            bytes_per_line = 3 * width
            
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
        # Clean up local camera resources
        for cap in self.captures:
            if cap is not None:
                cap.release()
                
        # Clean up ROS 2 resources
        self.ros_spin_timer.stop()
        self.display_timer.stop()
        if hasattr(self, 'ros_node') and self.ros_node:
             self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    # The class now expects the number of ROS cameras and the number of local cameras
    window = MultiCameraGUI(num_ros_cameras=5, num_local_cameras=NUM_LOCAL_CAMERAS) 
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()