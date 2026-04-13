#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.callback,
            10
        )

        self.get_logger().info("Camera feed streamer started")

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                cv2.imshow("Car Camera Feed", frame)
                cv2.waitKey(1)  
        except Exception as e:
            self.get_logger().error(f"Error in viewer callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
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
