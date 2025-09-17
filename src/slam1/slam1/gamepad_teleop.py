#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')

        # Publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Speed settings
        self.base_linear = 0.5    # m/s
        self.base_angular = 1.0   # rad/s
        self.speed_scale = 1.0

        # For DPAD edge detection
        self.last_dpad = 0.0  

        self.get_logger().info("🎮 Gamepad teleop started")
        self.get_logger().info("Use left stick to move, DPAD ↑/↓ to change sensitivity, A to stop.")

    def joy_callback(self, msg: Joy):
        # Stick axes
        linear_axis = msg.axes[1]   # forward/back
        angular_axis = msg.axes[0]  # left/right

        # DPAD vertical (usually axes[7])
        dpad_vertical = msg.axes[7]

        # Detect DPAD press (avoid repeats by checking last state)
        if dpad_vertical != self.last_dpad:
            if dpad_vertical == 1.0:   # DPAD UP
                self.speed_scale *= 1.2
                self.get_logger().info(f"🔼 Sensitivity increased: {self.speed_scale:.2f}x")
            elif dpad_vertical == -1.0: # DPAD DOWN
                self.speed_scale *= 0.8
                self.get_logger().info(f"🔽 Sensitivity decreased: {self.speed_scale:.2f}x")
        self.last_dpad = dpad_vertical

        # Emergency stop with A button
        if msg.buttons[0]:  # A button
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            self.get_logger().warn("🛑 Emergency STOP pressed!")
            return

        # Build Twist
        twist = Twist()
        twist.linear.x = linear_axis * self.base_linear * self.speed_scale
        twist.angular.z = angular_axis * self.base_angular * self.speed_scale

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
