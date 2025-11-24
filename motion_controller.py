#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        self.get_logger().info("✅ Motion Controller Node started")

        # === Internal state ===
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # yaw

        # === Waypoints for autonomous run ===
        self.waypoints = [
            (0.2, 0.0), # in meters
            (0.2, 0.2),
            (0.0, 0.2),
            (0.0, 0.0),
        ]
        self.current_goal_index = 0
        self.goal_x = None
        self.goal_y = None
        self.new_goal_received = False

        # Track last waypoint printed
        self.last_reached_index = -1

        # === Control parameters ===
        self.kp_linear = 0.1
        self.kp_angular = 0.3
        self.xy_tolerance = 0.05
        self.yaw_tolerance = math.radians(5)

        # === Subscribers ===
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)

        # === Publisher ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === Timers ===
        self.create_timer(0.05, self.control_loop)  # 20 Hz control
        self.create_timer(1.0, self.debug_loop)      # 1 Hz debug

    # ====================
    # CALLBACKS
    # ====================
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.new_goal_received = True
        self.get_logger().info(f"🎯 New manual goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    # ====================
    # DEBUG LOOP (1 Hz)
    # ====================
    def debug_loop(self):
        """Print slow debug info once per second."""
        if self.new_goal_received:
            goal_info = f"Manual goal → ({self.goal_x:.2f}, {self.goal_y:.2f})"
        else:
            if self.current_goal_index < len(self.waypoints):
                gx, gy = self.waypoints[self.current_goal_index]
                goal_info = f"Waypoint {self.current_goal_index+1} → ({gx:.2f}, {gy:.2f})"
            else:
                goal_info = "No more waypoints."

        self.get_logger().info(
            f"📍 Pos=({self.x:.2f}, {self.y:.2f}) θ={math.degrees(self.theta):.1f}° | {goal_info}"
        )

    # ====================
    # CONTROL LOOP
    # ====================
    def control_loop(self):
        # Select target
        if self.new_goal_received:
            target_x = self.goal_x
            target_y = self.goal_y
        else:
            if self.current_goal_index >= len(self.waypoints):
                self.publish_stop()
                return
            target_x, target_y = self.waypoints[self.current_goal_index]

        # Errors in world frame
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)

        if distance < self.xy_tolerance:
            self.publish_stop()

            if not self.new_goal_received:
                self.current_goal_index += 1
            return

        # ====== MECANUM CONTROL ======
        cmd = Twist()

        # --- 1) ROTATE ONLY TO ALIGN TO 0° ---
        if abs(self.theta) > self.yaw_tolerance:
            cmd.angular.z = -self.kp_angular * self.theta
        else:
            cmd.angular.z = 0.0

        # --- 2) Convert error into robot frame (for x/y control) ---
        ex = math.cos(-self.theta) * dx - math.sin(-self.theta) * dy
        ey = math.sin(-self.theta) * dx + math.cos(-self.theta) * dy

        # --- 3) Drive toward target in robot frame ---
        cmd.linear.x = self.kp_linear * ex
        cmd.linear.y = self.kp_linear * ey

        # Limit speeds
        cmd.linear.x = max(min(cmd.linear.x, 0.2), -0.2)
        cmd.linear.y = max(min(cmd.linear.y, 0.2), -0.2)
        cmd.angular.z = max(min(cmd.angular.z, 0.4), -0.4)

        self.cmd_pub.publish(cmd)



    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    # ====================
    # UTILITY
    # ====================
    @staticmethod
    def angle_diff(target, current):
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Motion Controller interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
