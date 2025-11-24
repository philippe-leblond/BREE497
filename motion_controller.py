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
            (0.0, 0.2), # in meters
            (0.2, 0.2),
            (0.2, 0.0),
            (0.0, 0.0),
        ]
        self.current_goal_index = 0
        self.goal_x = None
        self.goal_y = None
        self.new_goal_received = False

        # Track last waypoint printed
        self.last_reached_index = -1

        # === Control parameters ===
        self.kp_linear = 0.12
        self.kp_angular = 0.0   # keep zero while turning disabled
        # tighter waypoint tolerance (meters) and confirmation count
        self.xy_tolerance = 0.03
        self.arrival_required = 2   # require 4 consecutive control ticks (~0.2s at 20Hz)
        self._arrival_count = 0
        self.yaw_tolerance = math.radians(5)
        # minimum commanded travel speed to overcome motor deadzone (m/s)
        self.min_travel_speed = 0.02
        # on arrival, publish STOP for a few cycles so motor driver sees it
        self.post_arrival_stop_cycles = 4
        self._stop_cycles = 0

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

        # DEBUG: show raw odom values (precise numbers and frame ids)
        self.get_logger().debug(
            f"odom header.frame_id='{msg.header.frame_id}' child_frame_id='{msg.child_frame_id}' "
            f"pos=({self.x:.6f}, {self.y:.6f}, {msg.pose.pose.position.z:.6f}) "
            f"yaw={math.degrees(self.theta):.3f}° cov[0]={msg.pose.covariance[0]:.6g}"
        )

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
        # if we are in forced stop cycles (after arrival), keep publishing STOP
        if self._stop_cycles > 0:
            self.publish_stop()
            self._stop_cycles -= 1
            return

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

        # Per-axis deadband: if one axis is already within tolerance, zero it so we don't
        # generate small residual commands on that axis due to heading offsets.
        if abs(dx) <= self.xy_tolerance:
            dx = 0.0
        if abs(dy) <= self.xy_tolerance:
            dy = 0.0

        # Debounce arrival: require several consecutive ticks inside tolerance
        if distance <= self.xy_tolerance:
            self._arrival_count += 1
        else:
            self._arrival_count = 0

        if self._arrival_count >= self.arrival_required:
            # confirmed arrival
            self.get_logger().info(f"✅ Waypoint reached ({target_x:.2f}, {target_y:.2f}) after {self._arrival_count} checks")
            self.publish_stop()
            self._arrival_count = 0
            # force a few STOP publishes so motor driver receives them
            self._stop_cycles = self.post_arrival_stop_cycles
            if not self.new_goal_received:
                self.current_goal_index += 1
            return

        # ====== MECANUM CONTROL (NO ROTATION) ======
        cmd = Twist()

        # do not issue rotation commands while tuning base travel
        cmd.angular.z = 0.0

        # --- Convert error into robot frame (for x/y control) ---
        ex = math.cos(-self.theta) * dx - math.sin(-self.theta) * dy
        ey = math.sin(-self.theta) * dx + math.cos(-self.theta) * dy

        # --- Drive toward target in robot frame ---
        if abs(dx) <= self.xy_tolerance:
            ey = self.kp_linear * dy  # Only move in y if x is within tolerance
            ex = 0.0  # Zero out x error

        cmd.linear.x = self.kp_linear * ex
        cmd.linear.y = self.kp_linear * ey

        # Limit speeds
        cmd.linear.x = max(min(cmd.linear.x, 0.2), -0.2)
        cmd.linear.y = max(min(cmd.linear.y, 0.2), -0.2)

        # enforce a minimum commanded travel magnitude so motor driver doesn't treat as deadzone
        mag_cmd = max(abs(cmd.linear.x), abs(cmd.linear.y))
        if mag_cmd > 0.0 and mag_cmd < self.min_travel_speed:
            scale = self.min_travel_speed / mag_cmd
            cmd.linear.x *= scale
            cmd.linear.y *= scale

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
