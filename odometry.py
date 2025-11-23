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

        self.get_logger().info("🚀 Mecanum Odometry Node Started")

        qos = QoSProfile(depth=10)

        # === Subscribers ===
        self.create_subscription(Int32MultiArray, '/encoders', self.enc_callback, qos)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, qos)

        # === Publisher ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # === TF broadcaster ===
        self.tf_broadcaster = TransformBroadcaster(self)

        # === Robot parameters ===
        self.ticks_per_rev = 1855
        self.wheel_radius = 0.06/2
        self.base_length = 0.099
        self.base_width = 0.129

        # === State Variables ===
        self.last_encoder = None
        self.last_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # IMU
        self.imu_yaw_rate = 0.0

        # Debug print limiter
        self.last_debug = 0


    # ==============================
    # IMU CALLBACK
    # ==============================
    def imu_callback(self, msg: Imu):
        self.imu_yaw_rate = msg.angular_velocity.z


    # ==============================
    # ENCODER CALLBACK
    # ==============================
    def enc_callback(self, msg: Int32MultiArray):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0:
            return

        ticks = msg.data  # [FL, FR, RL, RR]

        # First message — store baseline
        if self.last_encoder is None:
            self.last_encoder = ticks
            self.get_logger().info(f"📥 First encoder message received: {ticks}")
            return

        # Δticks
        dticks = [ticks[i] - self.last_encoder[i] for i in range(4)]
        self.last_encoder = ticks

        # Wheel velocities
        wheel_linear = []
        for d in dticks:
            rev = d / self.ticks_per_rev
            dist = rev * (2 * math.pi * self.wheel_radius)
            wheel_linear.append(dist / dt)

        v_fl, v_fr, v_rl, v_rr = wheel_linear

        # LOW-PASS FILTER WHEEL SPEEDS  
        if not hasattr(self, "prev_wheel"):
            self.prev_wheel = [0, 0, 0, 0]

        alpha_filter = 0.3  # 0=strong smoothing, 1=no smoothing
        wheel_linear = [
            alpha_filter * wl + (1 - alpha_filter) * prev
            for wl, prev in zip(wheel_linear, self.prev_wheel)
        ]

        self.prev_wheel = wheel_linear

        # Mecanum kinematics
        L = self.base_length
        W = self.base_width

        vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        wz = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (L + W))

        # CLIP wz TO AVOID EXPLOSIONS 
        wz = max(min(wz, 3.0), -3.0)


        # 100% IMU yaw integration
        self.theta += self.imu_yaw_rate * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))


        # Integrate position
        dx = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        dy = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        self.x += dx
        self.y += dy

        # ======================
        # DEBUG PRINTS (5 Hz)
        # ======================
        if now - self.last_debug > 0.2:
            self.last_debug = now

            self.get_logger().info(
                f"\n=== 📡 ODOM DEBUG ===\n"
                f"Ticks: {ticks}\n"
                f"ΔTicks: {dticks}\n"
                f"Wheel v (m/s): FL={v_fl:.3f}, FR={v_fr:.3f}, "
                f"RL={v_rl:.3f}, RR={v_rr:.3f}\n"
                f"Robot Vel: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}\n"
                f"Pose: x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.1f}°"
            )

        # Publish odometry
        self.publish_odom(vx, vy, wz)


    # ==============================
    # PUBLISH ODOM MESSAGE
    # ==============================
    def publish_odom(self, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        # Velocities
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # TF broadcast
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
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
