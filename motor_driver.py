#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from rclpy.duration import Duration


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("⚙️ Motor Driver Node started (STOP only on arrival)")

        # --- Serial connection ---
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to ESP32: {e}")
            raise

        # --- Subscribe to /cmd_vel ---
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.get_logger().info("📡 Subscribed to /cmd_vel")

        # STOP spam prevention
        self.last_command = None

        # Speed limits (m/s)
        self.max_linear_speed = 0.10
        self.max_angular_speed = 0.05

        # Watchdog (log-only)
        self.last_command_time = self.get_clock().now()
        self.command_timeout = Duration(seconds=2.0)
        self.create_timer(0.5, self.watchdog_check)

    # ======================================================
    # CALLBACK: /cmd_vel
    # ======================================================
    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        command = "<STOP:0>"

        # Normalize
        lin_mag = max(abs(linear_x), abs(linear_y))
        lin_norm = min(1.0, lin_mag / max(self.max_linear_speed, 1e-6))
        ang_norm = min(1.0, abs(angular_z) / max(self.max_angular_speed, 1e-6))

        linear_speed = int(lin_norm * 255)
        angular_speed = int(ang_norm * 255)

        # --- DEADZONE COMPENSATION ---
        MIN_PWM = 120

        if linear_speed > 0:
            linear_speed = max(MIN_PWM, linear_speed)

        if angular_speed > 0:
            angular_speed = max(MIN_PWM, angular_speed)
            angular_speed = min(150, angular_speed)  # safe limit

        # Motion mode selection
        if abs(angular_z) > 0.1:
            command = f"<TURNLEFT:{angular_speed}>" if angular_z > 0 else f"<TURNRIGHT:{angular_speed}>"
        elif abs(linear_x) > 0.01:
            command = f"<FORWARD:{linear_speed}>" if linear_x > 0 else f"<BACKWARD:{linear_speed}>"
        elif abs(linear_y) > 0.01:
            command = f"<MOVERIGHT:{linear_speed}>" if linear_y > 0 else f"<MOVELEFT:{linear_speed}>"
        else:
            command = "<STOP:0>"

        # Avoid STOP spam
        if command == "<STOP:0>" and self.last_command == "<STOP:0>":
            return

        # Send only when changed
        if command != self.last_command:
            self.send_command(command)
            self.last_command = command

        # Track last time we got a movement command
        self.last_command_time = self.get_clock().now()

    # ======================================================
    # Watchdog (log-only)
    # ======================================================
    def watchdog_check(self):
        now = self.get_clock().now()
        if (now - self.last_command_time) > self.command_timeout:
            self.get_logger().warn("⏳ No /cmd_vel received for 2s")
        else:
            self.get_logger().debug("✔️ Watchdog OK")

    # ======================================================
    # SERIAL SEND
    # ======================================================
    def send_command(self, command: str):
        try:
            self.ser.write((command + "\n").encode())
            self.get_logger().info(f"➡️ Sent to ESP32: {command}")
        except Exception as e:
            self.get_logger().error(f"⚠️ Serial write error: {e}")

    # ======================================================
    # CLEANUP
    # ======================================================
    def destroy_node(self):
        self.get_logger().info("🛑 Sending STOP before shutdown")

        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(b"<STOP:0>\n")
                self.get_logger().info("➡️ STOP sent to ESP32")
        except Exception as e:
            self.get_logger().error(f"⚠️ Could not send STOP: {e}")

        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("🛑 Ctrl+C pressed — stopping robot")
        node.send_command("<STOP:0>")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
