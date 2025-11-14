import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


class ImagePublisher(Node):
    def __init__(self, camera_index, topic_name):
        """
        Initializes the ImagePublisher node, sets up the ROS 2 publisher and timer, 
        and opens the local video capture device.

        Args:
            camera_index (int): The system index (e.g., 0, 1, 2) of the local camera device to open.
            topic_name (str): The ROS 2 topic name where image messages will be published.
        """
        super().__init__('image_publisher_' + str(camera_index))
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.timer = self.create_timer(0.03, self.timer_callback)

        self.cap = cv2.VideoCapture(i, cv2.CAP_V4L2)

        #Checks to see if the camera even opens or not
        if not self.cap.isOpened():
            self.get_logger().error(f"FATAL: Failed to open camera index {camera_index}")

        self.bridge = CvBridge()
        self.get_logger().info(f"Publisher for camera {camera_index} on topic {topic_name}")

    def timer_callback(self):
        """
        Captures a frame from the local camera and publishes it as a ROS 2 Image message.
        """
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8') 
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    """
    The entry point for the ROS 2 ImagePublisher node.

    Initializes the ROS 2 context, parses command-line arguments (sys.argv) 
    to determine the local camera index and publishing topic, creates the 
    ImagePublisher node, and begins spinning the node to process callbacks. 
    It ensures a graceful shutdown on exit or interruption.

    Args:
        args (list[str], optional): Command-line arguments passed to the 
            rclpy context. Defaults to None.
    """
    rclpy.init(args=args)
    # Use CLI args for camera index & topic
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    topic_name = f"/camera{camera_index}/image_raw"
    node = ImagePublisher(camera_index, topic_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
