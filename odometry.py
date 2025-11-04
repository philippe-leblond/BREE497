import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler
import math
import time

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info("✅ Odometry node started")

        # Subscriptions
        self.encoder_sub = self.create_subscription(Int32MultiArray, '/encoders', self.encoder_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Internal state
        self.last_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_yaw = 0.0
        self.use_imu = True

        # Constants
        self.TICKS_PER_REV = 1000
        self.WHEEL_RADIUS = 0.03  # m
        self.WHEEL_BASE = 0.15  # m

    def encoder_callback(self, msg):
        try:
            data = msg.data
            self.get_logger().debug(f"Received encoder data: {data}")

            if len(data) < 2:
                self.get_logger().warn(f"Encoder message too short: {data}")
                return

            left_ticks, right_ticks = data[0], data[1]

            # Compute delta ticks
            delta_left = left_ticks - self.last_left_ticks
            delta_right = right_ticks - self.last_right_ticks
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks

            # Time step
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            if dt <= 0:
                self.get_logger().warn("Non-positive dt detected, skipping update")
                return

            # Distance calculation
            d_left = 2 * math.pi * self.WHEEL_RADIUS * (delta_left / self.TICKS_PER_REV)
            d_right = 2 * math.pi * self.WHEEL_RADIUS * (delta_right / self.TICKS_PER_REV)
            d_center = (d_left + d_right) / 2.0

            # Update theta
            self.theta += (d_right - d_left) / self.WHEEL_BASE

            # integrate gyro.z (rad/s)
            yaw = self.imu_yaw if self.use_imu else self.theta

            # Update x, y postition
            self.x += d_center * math.cos(yaw)
            self.y += d_center * math.sin(yaw)
            
            # Convert yaw to quaternion for publishing
            q = quaternion_from_euler(0, 0, yaw)

            # Debug messages
            self.get_logger().debug(f"Δticks: L={delta_left:.2f}, R={delta_right:.2f}, Δt={dt:.3f}s")
            self.get_logger().debug(f"Pose: x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.2f}°")

            # Publish odometry
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist()
            odom.twist.twist.linear.x = d_center / dt
            odom.twist.twist.angular.z = (d_right - d_left) / (self.WHEEL_BASE * dt)

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().error(f"Error in encoder_callback: {e}")

    def imu_callback(self, msg: Imu):
        try:
            # Time step for gyro integration
            current_time = self.get_clock().now()
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            self.last_imu_time = current_time
            if dt <= 0:
                return

            # Integrate yaw from angular velocity around Z
            self.imu_yaw += msg.angular_velocity.z * dt
            # Normalize between -pi and pi
            self.imu_yaw = math.atan2(math.sin(self.imu_yaw), math.cos(self.imu_yaw))

            self.get_logger().debug(
                f"Integrated IMU yaw: {math.degrees(self.imu_yaw):.2f}°  (Δt={dt:.3f}s)"
            )

        except Exception as e:
            self.get_logger().error(f"Error in imu_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    # Set logger to DEBUG for full visibility
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
