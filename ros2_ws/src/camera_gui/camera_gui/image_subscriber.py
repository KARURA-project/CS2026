import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node):
    """
    Subscribes to a single camera feed and displays it in an OpenCV window.
    """

    def __init__(self):
        super().__init__('image_subscriber')

        # Subscribe to one topic (you can change the topic if needed)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',       # <-- topic name
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info("Image subscriber started")

    def listener_callback(self, msg):
        # Convert ROS Image message → OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Show in GUI window
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

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
