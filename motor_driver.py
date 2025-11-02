#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from rclpy.duration import Duration


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("⚙️ Motor Driver Node with failsafe started")

        # --- Serial connection ---
        self.port = '/dev/ttyUSB0'  # Update this to your ESP32 port
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to {self.port}: {e}")
            raise

        # --- Subscribe to /safe_cmd_vel ---
        self.subscription = self.create_subscription(
            Twist,
            '/safe_cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # --- Failsafe variables ---
        self.last_command_time = self.get_clock().now()
        self.command_timeout = Duration(seconds=2.0)  # Stop if no command for 2 seconds
        self.last_command = None

        # --- Timer to check for command timeout ---
        self.create_timer(0.5, self.watchdog_check)

    # ======================================================
    # CALLBACK: Convert Twist → discrete motion command
    # ======================================================
    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        #command = "<STOP>"
        command = "<FORWARD>"


        if linear_x > 0.1:
            command = "<FORWARD>"
        elif linear_x < -0.1:
            command = "<BACKWARD>"
        elif linear_y > 0.1:
            command = "<RIGHT>"
        elif linear_y < -0.1:
            command = "<LEFT>"
        elif abs(angular_z) > 0.1:
            command = "<TURN_LEFT>" if angular_z > 0 else "<TURN_RIGHT>"

        # Only send new commands when they change
        if command != self.last_command:
            self.send_command(command)
            self.last_command = command

        # Update last message time
        self.last_command_time = self.get_clock().now()

    # ======================================================
    # FAILSAFE WATCHDOG
    # ======================================================
    def watchdog_check(self):
        """Stop the robot if no command has been received recently."""
        time_since_last = self.get_clock().now() - self.last_command_time
        if time_since_last > self.command_timeout:
            if self.last_command != "<STOP>":
                self.get_logger().warn("⏱️ No cmd_vel received for 2s — sending STOP")
                self.send_command("<STOP>")
                self.last_command = "<STOP>"

    # ======================================================
    # SERIAL SEND FUNCTION
    # ======================================================
    def send_command(self, command: str):
        try:
            self.ser.write((command + "\n").encode())
            self.get_logger().info(f"➡️ Sent command: {command}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Serial write error: {e}")

    # ======================================================
    # CLEANUP
    # ======================================================
    def destroy_node(self):
        self.get_logger().info("🛑 Closing serial port...")
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧹 Stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == '__main__':
    main()
