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
        self.initial_heading = None

        # === Waypoints for autonomous run ===
        self.waypoints = [
            (0.0, 0.1), # in meters
            (0.1, 0.1),
            (0.1, 0.0),
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
        self.kp_angular = 0.07   # keep zero while turning disabled
        # tighter waypoint tolerance (meters) and confirmation count
        self.xy_tolerance = 0.03
        self.arrival_required = 2   # require 4 consecutive control ticks (~0.2s at 20Hz)
        self._arrival_count = 0
        self.yaw_drift_threshold = math.radians(8)
        # minimum commanded travel speed to overcome motor deadzone (m/s)
        self.min_travel_speed = 0.02
        self.max_turn_speed = 0.4  # rad/s
        # on arrival, publish STOP for a few cycles so motor driver sees it
        self.post_arrival_stop_cycles = 4
        self._stop_cycles = 0

        

        # === Subscribers ===
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)

        # === Publisher ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === Timers ===
        self.create_timer(1.00, self.control_loop)  # 5 Hz control
        self.create_timer(1.00, self.debug_loop)    # 5 Hz debug

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

        if self.initial_heading is None:
            self.initial_heading = self.theta
            self.get_logger().info(f"🎯 Initial heading locked at {math.degrees(self.theta):.2f}°")


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
    def publish_stop(self):
        """Send a STOP command to the motor driver."""
        stop = Twist()
        stop.linear.x = 0.0
        stop.linear.y = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)
        self.get_logger().info("🛑 Published STOP Twist")

    # ====================
    # CONTROL LOOP
    # ====================
    def control_loop(self):

        # Wait until odom callback initializes heading
        if self.initial_heading is None:
            return

        # ========= SELECT TARGET =========
        if self.new_goal_received:
            # manual goal mode
            target_x = self.goal_x
            target_y = self.goal_y

        else:
            # autonomous waypoint mode
            if self.current_goal_index >= len(self.waypoints):
                # === All waypoints completed ===
                self.get_logger().info("🎉 All waypoints complete — sending STOP")
                self.publish_stop()
                return

            # safe to index the waypoint
            target_x, target_y = self.waypoints[self.current_goal_index]


        # ========= POSITION ERROR (WORLD FRAME) =========
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)

        # ---- ARRIVAL DEBOUNCE ----
        if distance <= self.xy_tolerance:
            self._arrival_count += 1
        else:
            self._arrival_count = 0

        # ========= ADVANCE TO NEXT WAYPOINT (NO STOPPING) =========
        if self._arrival_count >= self.arrival_required:
            self.get_logger().info(
                f"➡️ Advancing: reached waypoint {self.current_goal_index+1} "
                f"({target_x:.2f}, {target_y:.2f})"
            )
            self._arrival_count = 0

            if not self.new_goal_received:
                self.current_goal_index += 1

            return  # Immediately compute next waypoint on next loop tick

        # ========= ROBOT-FRAME ERROR =========
        ex = math.cos(-self.theta) * dx - math.sin(-self.theta) * dy
        ey = math.sin(-self.theta) * dx + math.cos(-self.theta) * dy

        # ========= SIMPLE P CONTROL =========
        vx_cmd = self.kp_linear * ex
        vy_cmd = self.kp_linear * ey

        # ========= LIMIT SPEEDS =========
        vx_cmd = max(min(vx_cmd, 0.2), -0.2)
        vy_cmd = max(min(vy_cmd, 0.2), -0.2)

        # ========= DEADZONE COMPENSATION =========
        mag_cmd = max(abs(vx_cmd), abs(vy_cmd))
        if mag_cmd > 0.0 and mag_cmd < self.min_travel_speed:
            scale = self.min_travel_speed / mag_cmd
            vx_cmd *= scale
            vy_cmd *= scale

         # ===========================================
        #  YAW HOLD MODE (ONLY CORRECTS DRIFT)
        # ===========================================
        yaw_error = self.angle_diff(self.initial_heading, self.theta)

        if abs(yaw_error) < self.yaw_drift_threshold:
            ang_cmd = 0.0
        else:
            ang_cmd = self.kp_angular * yaw_error
            ang_cmd = max(min(ang_cmd, self.max_turn_speed), -self.max_turn_speed)

        # ========= SEND CMD_VEL =========
        cmd = Twist()
        cmd.angular.z = ang_cmd
        cmd.linear.x = vx_cmd
        cmd.linear.y = vy_cmd
        self.cmd_pub.publish(cmd)

        # ========= DEBUG =========
        # self.get_logger().info(
        #    f"[CTRL] wp_idx={self.current_goal_index} "
        #    f"target=({target_x:.3f},{target_y:.3f}) "
        #    f"pos=({self.x:.3f},{self.y:.3f}) "
        #    f"dx={dx:.3f} dy={dy:.3f} dist={distance:.3f} "
        #    f"ex={ex:.3f} ey={ey:.3f} "
        #    f"cmd_x={vx_cmd:.3f} cmd_y={vy_cmd:.3f} "
        #    f"arr_cnt={self._arrival_count}"
        #)
    



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