#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

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
            (0.5, 0.0),
            (1.0, 0.5),
            (1.5, 0.0),
            (1.0, -0.5),
            (0.5, 0.0)
        ]
        self.current_goal_index = 0
        self.goal_x = None
        self.goal_y = None
        self.new_goal_received = False

        # === Control parameters ===
        self.kp_linear = 1.0    # proportional gain for linear velocity
        self.kp_angular = 2.0   # proportional gain for angular velocity
        self.xy_tolerance = 0.05  # meters
        self.yaw_tolerance = math.radians(5)  # radians

        # === Subscribers ===
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)

        # === Publisher ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === Timer for control loop ===
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    # ====================
    # CALLBACKS
    # ====================
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.new_goal_received = True
        self.get_logger().info(f"🎯 New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    # ====================
    # CONTROL LOOP
    # ====================
    def control_loop(self):
        # Select current target
        if self.new_goal_received:
            target_x = self.goal_x
            target_y = self.goal_y
        else:
            if self.current_goal_index >= len(self.waypoints):
                self.publish_stop()
                return
            target_x, target_y = self.waypoints[self.current_goal_index]

        # Compute errors
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.angle_diff(angle_to_goal, self.theta)

        # Debug
        self.get_logger().debug(f"Current pos: ({self.x:.2f}, {self.y:.2f}), Goal: ({target_x:.2f}, {target_y:.2f}), Dist: {distance:.2f}, Angle error: {math.degrees(angle_error):.2f}")

        # Check if goal reached
        if distance < self.xy_tolerance:
            if not self.new_goal_received:
                self.current_goal_index += 1
                self.get_logger().info(f"✅ Reached waypoint {self.current_goal_index}")
            self.publish_stop()
            return

        # Proportional control
        vx = self.kp_linear * distance
        vy = 0.0  # mecanum will handle lateral movement if desired
        wz = self.kp_angular * angle_error

        # Limit max velocities
        vx = max(min(vx, 0.5), -0.5)
        wz = max(min(wz, 1.0), -1.0)

        # Publish command
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # ====================
    # UTILITY
    # ====================
    @staticmethod
    def angle_diff(target, current):
        """Compute minimal angle difference [-pi, pi]."""
        diff = target - current
        while diff > math.pi:
            diff -= 2*math.pi
        while diff < -math.pi:
            diff += 2*math.pi
        return diff

def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Motion Controller interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
