#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from rclpy.duration import Duration


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("⚙️ Motor Driver Node started (rotation disabled)")

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
        # ignore angular_z while rotation disabled

        command = "<STOP:0>"

        # Normalize using dominant axis magnitude
        lin_mag = max(abs(linear_x), abs(linear_y))
        lin_norm = min(1.0, lin_mag / max(self.max_linear_speed, 1e-6))

        linear_speed = int(lin_norm * 255)

        # --- DEADZONE COMPENSATION ---
        MIN_PWM = 180
        MOTION_DEADZONE = 0.005

        if linear_speed > 0:
            linear_speed = max(MIN_PWM, linear_speed)

        # Choose dominant axis to send single-direction commands
        if lin_mag < MOTION_DEADZONE:
            command = "<STOP:0>"
        else:
            if abs(linear_x) >= abs(linear_y):
                command = f"<FORWARD:{linear_speed}>" if linear_x > 0 else f"<BACKWARD:{linear_speed}>"
            else:
                command = f"<MOVERIGHT:{linear_speed}>" if linear_y > 0 else f"<MOVELEFT:{linear_speed}>"

        # Avoid STOP spam
        if command == "<STOP:0>" and self.last_command == "<STOP:0>":
            # don't resend STOP, but refresh last_command_time so watchdog knows we're still receiving /cmd_vel
            self.last_command_time = self.get_clock().now()
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
