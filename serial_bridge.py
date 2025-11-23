#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu


def parse_serial_line(line: str):
    """Convert <key:value,key:value,...> into dict"""
    if line.startswith("<") and line.endswith(">"):
        try:
            line = line[1:-1]  # strip <>
            return {k: v for k, v in (pair.split(":") for pair in line.split(","))}
        except Exception:
            return None
    return None


class Navigation(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # === Serial setup ===
        self.get_logger().info("Starting serial_bridge_node...")
        try:
            self.ser_encoders = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Connected to /dev/ttyUSB0 ✅ (Encoders)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to /dev/ttyUSB0: {e}")
            raise

        try:
            self.ser_sensors = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            self.get_logger().info("Connected to /dev/ttyUSB1 ✅ (Sensors)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to /dev/ttyUSB1: {e}")
            raise

        # === Publishers (ONLY encoders + IMU kept) ===
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoders', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)

        self.get_logger().info("✅ Publishers initialized (encoders + IMU only)")
        
        # === Threads ===
        threading.Thread(target=self.read_encoders, daemon=True).start()
        threading.Thread(target=self.read_sensors, daemon=True).start()
        self.get_logger().info("🚀 Reader threads started for both ESP32s")

    # ====================
    # ENCODERS THREAD
    # ====================
    def read_encoders(self):
        self.get_logger().info("📟 Encoder reader thread active")
        while rclpy.ok():
            try:
                line = self.ser_encoders.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                data = parse_serial_line(line)
                if not data:
                    continue

                enc_msg = Int32MultiArray()
                enc_msg.data = [
                    int(data.get('ENC_FL', 0)),
                    int(data.get('ENC_FR', 0)),
                    int(data.get('ENC_RL', 0)),
                    int(data.get('ENC_RR', 0))
                ]
                self.encoder_pub.publish(enc_msg)
                self.get_logger().info(f"ENC: {enc_msg.data}")


            except Exception as e:
                self.get_logger().warn(f"⚠️ Encoder read error: {e}")

    # ====================
    # SENSORS THREAD — ONLY IMU NOW
    # ====================
    def read_sensors(self):
        self.get_logger().info("🧭 IMU reader thread active")
        while rclpy.ok():
            try:
                line = self.ser_sensors.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                data = parse_serial_line(line)
                if not data:
                    continue

                # --- IMU only ---
                imu_msg = Imu()
                imu_msg.angular_velocity.x = float(data.get('GX', 0.0))
                imu_msg.angular_velocity.y = float(data.get('GY', 0.0))
                imu_msg.angular_velocity.z = float(data.get('GZ', 0.0))
                imu_msg.linear_acceleration.x = float(data.get('AX', 0.0))
                imu_msg.linear_acceleration.y = float(data.get('AY', 0.0))
                imu_msg.linear_acceleration.z = float(data.get('AZ', 0.0))

                self.imu_pub.publish(imu_msg)
                self.get_logger().info(
                    f"IMU: gyro=({imu_msg.angular_velocity.x:.2f}, "
                    f"{imu_msg.angular_velocity.y:.2f}, "
                    f"{imu_msg.angular_velocity.z:.2f}), "
                    f"acc=({imu_msg.linear_acceleration.x:.2f}, "
                    f"{imu_msg.linear_acceleration.y:.2f}, "
                    f"{imu_msg.linear_acceleration.z:.2f})"
                )



            except Exception as e:
                self.get_logger().warn(f"⚠️ IMU read error: {e}")

    # ====================
    # CLEAN SHUTDOWN
    # ====================
    def destroy_node(self):
        self.get_logger().info("🛑 Closing serial ports...")
        try:
            if hasattr(self, 'ser_encoders') and self.ser_encoders.is_open:
                self.ser_encoders.close()
            if hasattr(self, 'ser_sensors') and self.ser_sensors.is_open:
                self.ser_sensors.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing serial: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = Navigation()
    node.get_logger().info("✅ Serial bridge node running (IMU + encoders only)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧹 Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == '__main__':
    main()
