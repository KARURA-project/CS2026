import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class ImagePublisher(Node):
    def __init__(self, camera_index, topic_name):
        super().__init__('image_publisher_' + str(camera_index))
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.cap = cv2.VideoCapture(camera_index)
        self.bridge = CvBridge()
        self.get_logger().info(f"Publisher for camera {camera_index} on topic {topic_name}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
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
