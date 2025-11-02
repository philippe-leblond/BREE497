#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class MecanumKinematicsNode(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics_node')
        self.get_logger().info("✅ Mecanum Kinematics Node started")

        # === Robot parameters ===
        self.L = 0.18  # distance between front and rear wheels (m)
        self.W = 0.15  # distance between left and right wheels (m)
        self.r = 0.033  # wheel radius (m)

        # === Subscriber & Publisher ===
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.wheel_pub = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Debug
        self.get_logger().debug(f"Received cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")

        # === Mecanum wheel kinematics ===
        # Wheel speed calculation (rad/s)
        # FL = Front Left, FR = Front Right, RL = Rear Left, RR = Rear Right
        fl = (1 / self.r) * (vx - vy - (self.L + self.W) * wz)
        fr = (1 / self.r) * (vx + vy + (self.L + self.W) * wz)
        rl = (1 / self.r) * (vx + vy - (self.L + self.W) * wz)
        rr = (1 / self.r) * (vx - vy + (self.L + self.W) * wz)

        # Publish wheel speeds
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [fl, fr, rl, rr]
        self.wheel_pub.publish(wheel_msg)

        # Debug
        self.get_logger().debug(f"Wheel speeds published: FL={fl:.2f}, FR={fr:.2f}, RL={rl:.2f}, RR={rr:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematicsNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Mecanum Kinematics interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
