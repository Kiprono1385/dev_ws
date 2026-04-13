import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image as PILImage
import cv2


class InferenceEngine(Node):
    def __init__(self):
        super().__init__('inference_engine')
        self.sub = self.create_subscription(Image, '/potato_image', self.callback, 10)
        self.pub = self.create_publisher(String, '/inference_result', 10)
        self.bridge = CvBridge()

        self.classes = ["Early_blight", "Healthy", "Late_blight"]

        # ✅ FIXED: Use absolute path for model
        package_path = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(package_path, '..', 'models', 'model_v2.pth')
        model_path = os.path.abspath(model_path)
        self.get_logger().info(f"Loading model from: {model_path}")

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = models.resnet18(weights=None)
        self.model.fc = nn.Linear(self.model.fc.in_features, len(self.classes))

        # Load model safely
        try:
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.eval()
            self.get_logger().info("✅ Model loaded successfully.")
        except FileNotFoundError:
            self.get_logger().error(f"❌ Model file not found at {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406],
                                 [0.229, 0.224, 0.225])
        ])

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb_image)
        img_t = self.transform(pil_img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            outputs = self.model(img_t)
            probs = torch.softmax(outputs, dim=1)
            pred_idx = torch.argmax(probs, dim=1).item()

        result = f"{self.classes[pred_idx]} ({probs[0][pred_idx]:.4f})"
        self.get_logger().info(f"Inference Result: {result}")
        self.pub.publish(String(data=result))


def main():
    rclpy.init()
    node = InferenceEngine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
