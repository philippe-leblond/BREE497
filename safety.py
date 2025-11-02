#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Twist

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.get_logger().info("✅ Safety Node started")

        # Parameters
        self.min_distance = 0.15  # meters — stop if object closer than this
        self.line_trigger = 1     # value that means danger line detected (adjust for your line sensors)

        # State variables
        self.last_cmd_vel = Twist()
        self.ultrasonic_data = []
        self.line_data = []

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, '/ultrasonic', self.ultrasonic_callback, 10)
        self.create_subscription(Int32MultiArray, '/line_sensors', self.line_callback, 10)

        # Publisher
        self.safe_pub = self.create_publisher(Twist, '/safe_cmd_vel', 10)

        # Timer for safety check
        self.create_timer(0.1, self.safety_check)  # 10 Hz

    def cmd_vel_callback(self, msg: Twist):
        """Store the latest commanded velocity"""
        self.last_cmd_vel = msg

    def ultrasonic_callback(self, msg: Float32MultiArray):
        """Update ultrasonic readings"""
        self.ultrasonic_data = msg.data

    def line_callback(self, msg: Int32MultiArray):
        """Update line sensor readings"""
        self.line_data = msg.data

    def safety_check(self):
        """Continuously evaluate sensor data and decide safe velocity"""
        danger = False

        # === Ultrasonic safety check ===
        if self.ultrasonic_data:
            min_dist = min(self.ultrasonic_data)
            if min_dist < self.min_distance:
                self.get_logger().warn(f"🚨 Obstacle detected at {min_dist:.2f} m — STOPPING")
                danger = True

        # === Line sensor safety check ===
        if self.line_data:
            if any(val == self.line_trigger for val in self.line_data):
                self.get_logger().warn("⚠️ Line sensor triggered — STOPPING")
                danger = True

        # === Publish safe velocity ===
        safe_cmd = Twist()
        if not danger:
            safe_cmd = self.last_cmd_vel  # forward motion command
        else:
            # Emergency stop
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.angular.z = 0.0

        self.safe_pub.publish(safe_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Safety Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
