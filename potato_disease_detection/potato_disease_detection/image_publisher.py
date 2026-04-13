import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class ImagePublisher(Node):
    def __init__(self, image_path):
        super().__init__('image_publisher')
        self.pub = self.create_publisher(Image, '/potato_image', 10)
        self.bridge = CvBridge()
        self.image_path = image_path
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_image)

    def publish_image(self):
        img = cv2.imread(self.image_path)
        if img is None:
            self.get_logger().error(f"Failed to load image: {self.image_path}")
            return
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(msg)
        self.get_logger().info(f"Published image: {self.image_path}")

def main():
    rclpy.init()
    image_path = sys.argv[1] if len(sys.argv) > 1 else "./potato_disease_detection/images/late.jpg"
    node = ImagePublisher(image_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
