#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster

import math
import time

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu


class MecanumOdometry(Node):
    def __init__(self):
        super().__init__('mecanum_odometry')

        self.get_logger().info("🚀 Mecanum Odometry Node (custom kinematics for your wiring) started")

        qos = QoSProfile(depth=10)

        # === Subscribers ===
        self.create_subscription(Int32MultiArray, '/encoders', self.enc_callback, qos)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, qos)

        # === Publisher ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # === TF broadcaster ===
        self.tf_broadcaster = TransformBroadcaster(self)

        # === Robot parameters ===
        self.ticks_per_rev = 1855.0
        self.wheel_radius = 0.03       # meters
        self.wheel_circ = math.pi * 0.06
        self.base_length = 0.099
        self.base_width  = 0.129

        # Lateral calibration factor (tune if strafe distance is off)
        self.strafe_scale = 1.0

        # === State variables ===
        self.last_encoder = None
        self.last_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # yaw

        # IMU yaw rate (rad/s)
        self.imu_yaw_rate = 0.0

        # Debug limiter
        self.last_debug = 0.0

    # =========================================================
    # IMU CALLBACK
    # =========================================================
    def imu_callback(self, msg: Imu):
        # we’ll integrate this for theta
        self.imu_yaw_rate = msg.angular_velocity.z

    # =========================================================
    # ENCODER CALLBACK
    # =========================================================
    def enc_callback(self, msg: Int32MultiArray):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0:
            return

        ticks = msg.data  # [FL, FR, RL, RR]

        # First encoder set
        if self.last_encoder is None:
            self.last_encoder = ticks
            self.get_logger().info(f"📥 First encoder ticks: {ticks}")
            return

        # === Compute delta ticks with wrap ===
        def diff_wrap(n, o):
            diff = int(n) - int(o)
            if abs(diff) > (1 << 31):   # 32-bit wrap
                diff -= (1 << 32) if diff > 0 else -(1 << 32)
            return diff

        dticks = [diff_wrap(ticks[i], self.last_encoder[i]) for i in range(4)]
        self.last_encoder = ticks

        # Filter bad dt
        if dt < 1e-5 or dt > 0.5:
            self.get_logger().warning(f"Ignoring encoder update dt={dt:.5f}s")
            return

        # === Convert ticks → meters ===
        raw = [(d / self.ticks_per_rev) * self.wheel_circ for d in dticks]
        s_fl, s_fr, s_rl, s_rr = raw

        # Reject crazy spikes
        if any(abs(r) > 0.5 for r in raw):
            self.get_logger().warning(f"⚠ Spike filtered: {raw}")
            return

        # === MECANUM KINEMATICS CUSTOMIZED TO YOUR PATTERNS ===
        # Your real encoder patterns:
        #  - Forward = [+ + + +]
        #  - Right   = [- + - +]
        #  - Left    = [+ - + -]
        #
        # We choose:
        #   dx_body = (s_fl + s_fr + s_rl + s_rr) / 4
        #   dy_body = (-s_fl + s_fr - s_rl + s_rr) / 4 * strafe_scale
        #
        # This gives:
        #   Forward: dx>0, dy=0
        #   Right:   dx≈0, dy>0
        #   Left:    dx≈0, dy<0

        dx_body = (s_fl + s_fr + s_rl + s_rr) / 4.0
        dy_body = self.strafe_scale * (-s_fl + s_fr - s_rl + s_rr) / 4.0

        # Use IMU for rotation (more reliable than encoders with your wiring)
        dtheta = self.imu_yaw_rate * dt

        # === Transform to world frame ===
        theta_mid = self.theta + dtheta * 0.5

        dx = math.cos(theta_mid) * dx_body - math.sin(theta_mid) * dy_body
        dy = math.sin(theta_mid) * dx_body + math.cos(theta_mid) * dy_body

        # === Update odometry ===
        self.x += dx
        self.y += dy
        self.theta = math.atan2(
            math.sin(self.theta + dtheta),
            math.cos(self.theta + dtheta)
        )

        vx = dx / dt
        vy = dy / dt
        wz = dtheta / dt

        # === Debug output ===
        if now - self.last_debug > 0.25:
            self.last_debug = now
            self.get_logger().info(
                f"\n=== 📡 ODOM DEBUG ===\n"
                f"Ticks: {ticks}\n"
                f"ΔTicks: {dticks}\n"
                f"raw(m): {['{:.4f}'.format(r) for r in raw]}\n"
                f"dx_body={dx_body:.4f}, dy_body={dy_body:.4f}, dtheta={dtheta:.4f}\n"
                f"Vel: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}\n"
                f"Pose: x={self.x:.3f} y={self.y:.3f} θ={math.degrees(self.theta):.2f}°"
            )

        self.publish_odom(vx, vy, wz)

    # =========================================================
    # ODOM PUBLISHER
    # =========================================================
    def publish_odom(self, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # === TF ===
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
