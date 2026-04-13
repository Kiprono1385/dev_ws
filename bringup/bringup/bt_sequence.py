#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import time
import yaml


class BTSequenceNode(Node):
    def __init__(self):
        super().__init__('bt_sequence_node')

        # ✅ Absolute path to your YAML file
        yaml_path = "/home/brian/dev_ws/src/bringup/config/poses.yaml"

        # Load waypoint data
        try:
            with open(yaml_path, 'r') as f:
                self.points = yaml.safe_load(f)
            self.get_logger().info(f"✅ Loaded waypoint YAML from {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YAML: {e}")
            raise e

        # Nav2 Action Client
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to deposit command (color detection result)
        qos = QoSProfile(depth=10)
        self.deposit_color = None
        self.create_subscription(String, '/deposit_command', self.deposit_callback, qos)

        # ✅ Publisher to control tipper
        self.tipper_pub = self.create_publisher(String, '/tipper_cmd', 10)

    def deposit_callback(self, msg: String):
        """Store the deposit command from color detection node."""
        self.deposit_color = msg.data.strip()
        self.get_logger().info(f"📩 Received deposit command: {self.deposit_color}")

    def send_goal_blocking(self, target):
        """Send goal to Nav2 and block until success/failure."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target['position']['x']
        goal_msg.pose.pose.position.y = target['position']['y']
        goal_msg.pose.pose.orientation.z = target['orientation']['z']
        goal_msg.pose.pose.orientation.w = target['orientation']['w']

        # Wait for server
        self.get_logger().info("⏳ Waiting for Nav2 action server...")
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Nav2 action server not available!")
            return False

        # Send goal
        self.get_logger().info(f"🚀 Navigating to: {target}")
        future = self._nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by Nav2!")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("✅ Reached goal successfully")
            return True
        else:
            self.get_logger().warn(f"⚠️ Navigation failed with status {result.status}")
            return False

    def activate_tipper(self, duration=5):
        """Activate tipper to unload, then deactivate."""
        self.get_logger().info("🔄 Activating tipper for unloading...")
        on_msg = String()
        on_msg.data = "ON"
        self.tipper_pub.publish(on_msg)

        time.sleep(duration)

        off_msg = String()
        off_msg.data = "OFF"
        self.tipper_pub.publish(off_msg)
        self.get_logger().info("✅ Tipper deactivated after unloading")

    def run_sequence(self):
        """Run the navigation behavior tree sequence."""
        self.get_logger().info("🚦 Starting navigation behavior sequence...")

        # 1️⃣ Plant Detection Station
        if self.send_goal_blocking(self.points['plant_detection']):
            self.get_logger().info("🌿 Arrived at Plant Detection Station — waiting 5 seconds")
            time.sleep(5)

        # 2️⃣ Color Detection Station
        if self.send_goal_blocking(self.points['color_detection']):
            self.get_logger().info("🎨 Arrived at Color Detection Station — waiting for color signal")
            time.sleep(5)

        # 3️⃣ Decide Deposit Target
        deposit_target = None
        if self.deposit_color and "Red" in self.deposit_color:
            deposit_target = self.points['red_deposit']
            self.get_logger().info("🟥 Color Detected → Going to RED Deposit Area")
        elif self.deposit_color and "Blue" in self.deposit_color:
            deposit_target = self.points['blue_deposit']
            self.get_logger().info("🟦 Color Detected → Going to BLUE Deposit Area")
        else:
            deposit_target = self.points['blue_deposit']
            self.get_logger().warn("⚠️ No color detected, defaulting to BLUE deposit")

        # 4️⃣ Navigate to Deposit Area
        if self.send_goal_blocking(deposit_target):
            self.get_logger().info("📦 At Deposit Area — unloading...")
            self.activate_tipper(duration=5)

        self.get_logger().info("🏁 Behavior sequence completed. Robot stopped at deposit area.")


def main(args=None):
    rclpy.init(args=args)
    node = BTSequenceNode()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
