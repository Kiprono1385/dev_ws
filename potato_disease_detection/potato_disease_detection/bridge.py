#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class CompressedToRawBridge(Node):
    def __init__(self):
        super().__init__('compressed_to_raw_bridge')

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.callback,
            10
        )

        self.pub = self.create_publisher(Image, '/potato_image', 10)
        self.bridge = CvBridge()

        self.get_logger().info("Camera -> Model bridge node started.")

    def callback(self, msg):
        try:
            # Decode JPEG from msg.data
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error("Failed to decode compressed image")
                return

            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header = msg.header  # preserve timestamp
            self.pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedToRawBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
