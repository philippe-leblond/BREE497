#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from rclpy.duration import Duration


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("⚙️ Motor Driver Node started (burst rotation enabled)")

        # --- Serial connection ---
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to ESP32: {e}")
            raise

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.get_logger().info("📡 Subscribed to /cmd_vel")

        # STOP spam prevention
        self.last_command = None

        # Speed limits (m/s)
        self.max_linear_speed = 0.10
        self.max_angular_speed = 1.0  # rad/s

        # === ROTATION BURST PARAMETERS ===
        self.turn_state = "IDLE"        # IDLE → BURST → PAUSE → IDLE ...
        self.turn_burst_duration = 0.12  # seconds turning
        self.turn_pause_duration = 0.10  # seconds waiting
        self.last_turn_time = self.get_clock().now()
        self.active_turn_command = None  # keeps TURNLEFT or TURNRIGHT during burst

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

        # Normalize using dominant axis magnitude
        lin_mag = max(abs(linear_x), abs(linear_y))
        ang_mag = abs(angular_z)

        lin_norm = min(1.0, lin_mag / max(self.max_linear_speed, 1e-6))
        ang_norm = min(1.0, ang_mag / max(self.max_angular_speed, 1e-6))

        linear_speed = int(lin_norm * 255)
        angular_speed = int(ang_norm * 255)

        # --- DEADZONE COMPENSATION ---
        MIN_PWM = 100
        MOTION_DEADZONE = 0.005
        TURN_DEADZONE = 0.01  # rad/s threshold to start turning

        if linear_speed > 0:
            linear_speed = max(MIN_PWM, linear_speed)
        if angular_speed > 0:
            angular_speed = max(MIN_PWM, angular_speed)

        # ======================================================
        #     BURST-BASED ROTATION SYSTEM
        # ======================================================
        now = self.get_clock().now()
        dt = (now - self.last_turn_time).nanoseconds / 1e9

        if ang_mag >= TURN_DEADZONE:
            # BEGIN ROTATION CORRECTION
            if self.turn_state == "IDLE":
                # start a new burst
                self.active_turn_command = (
                    f"<TURNLEFT:{angular_speed}>"
                    if angular_z > 0 else
                    f"<TURNRIGHT:{angular_speed}>"
                )
                command = self.active_turn_command
                self.turn_state = "BURST"
                self.last_turn_time = now

            elif self.turn_state == "BURST":
                # continue burst until timeout
                if dt < self.turn_burst_duration:
                    command = self.active_turn_command
                else:
                    command = "<STOP:0>"
                    self.turn_state = "PAUSE"
                    self.last_turn_time = now

            elif self.turn_state == "PAUSE":
                # hold STOP until pause expires
                if dt < self.turn_pause_duration:
                    command = "<STOP:0>"
                else:
                    self.turn_state = "IDLE"
                    # do NOT immediately restart turning; wait for next callback
                    command = "<STOP:0>"

        else:
            # Not rotating → reset state
            self.turn_state = "IDLE"
            self.active_turn_command = None

            # Linear motion choices
            if lin_mag < MOTION_DEADZONE:
                command = "<STOP:0>"
            else:
                if abs(linear_x) >= abs(linear_y):
                    command = f"<FORWARD:{linear_speed}>" if linear_x > 0 else f"<BACKWARD:{linear_speed}>"
                else:
                    command = f"<MOVERIGHT:{linear_speed}>" if linear_y > 0 else f"<MOVELEFT:{linear_speed}>"

        # Avoid STOP spam
        if command == "<STOP:0>" and self.last_command == "<STOP:0>":
            self.last_command_time = now
            return

        # Send only on change
        if command != self.last_command:
            self.send_command(command)
            self.last_command = command

        self.last_command_time = now

    # ======================================================
    # Watchdog
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
