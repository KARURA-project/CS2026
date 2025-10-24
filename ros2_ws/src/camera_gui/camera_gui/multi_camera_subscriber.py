import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class MultiCameraSubscriber(Node):
    """
    Subscribes to multiple camera topics and displays them in a single GUI window.
    """

    def __init__(self, num_cameras=5):
        super().__init__('multi_camera_subscriber')
        self.bridge = CvBridge()
        self.num_cameras = num_cameras

        # Initialize placeholders for frames
        self.frames = [np.zeros((240, 320, 3), dtype=np.uint8) for _ in range(num_cameras)]

        # Subscribe to each camera topic
        for i in range(num_cameras):
            topic = f'/camera{i}/image_raw'
            self.create_subscription(Image, topic, self.make_callback(i), 10)
            self.get_logger().info(f"Subscribed to {topic}")

        # Timer to refresh display every ~30ms
        self.timer = self.create_timer(0.03, self.display_all)

    def make_callback(self, index):
        """Returns a callback function bound to a specific camera index."""
        def callback(msg):
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frames[index] = frame
        return callback

    def display_all(self):
        """Display all camera feeds in a single OpenCV window."""
        try:
            # Resize all frames for consistent display
            resized = [cv2.resize(f, (320, 240)) for f in self.frames]

            # Combine into a grid (3 on top row, 2 on bottom)
            top = cv2.hconcat(resized[0:3])
            bottom_row = resized[3:5]
            if len(bottom_row) < 3:  # pad with black if less than 3
                bottom_row += [np.zeros_like(resized[0])] * (3 - len(bottom_row))
            bottom = cv2.hconcat(bottom_row)
            combined = cv2.vconcat([top, bottom])

            cv2.imshow("Multi-Camera Feed", combined)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Display error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraSubscriber(num_cameras=5)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
