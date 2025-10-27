import rclpy
from rclpy.node import Node
import serial
import threading

from std_msgs.msg import Int32, Float32, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray, Int32MultiArray

def parse_serial_line(line: str):
    """Convert <key:value,key:value,...> into dict"""
    if line.startswith("<") and line.endswith(">"):
        try:
            line = line[1:-1]  # strip <>
            return {k: v for k, v in (pair.split(":") for pair in line.split(","))}
        except Exception:
            return None
    return None

class navigation(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # Adjust serial ports for your setup
        self.ser_encoders = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser_sensors  = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        # Publishers
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'encoders', 10)
        self.ultrasonic_pub = self.create_publisher(Float32MultiArray, 'ultrasonic', 10)
        self.line_pub = self.create_publisher(Int32MultiArray, 'line_sensors', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)

        # Launch threads for both ESP32 serials
        threading.Thread(target=self.read_encoders, daemon=True).start()
        threading.Thread(target=self.read_sensors, daemon=True).start()

    def read_encoders(self):
        while rclpy.ok():
            try:
                line = self.ser_encoders.readline().decode().strip()
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

            except Exception as e:
                self.get_logger().warn(f"Encoder read error: {e}")

    def read_sensors(self):
        while rclpy.ok():
            try:
                line = self.ser_sensors.readline().decode().strip()
                data = parse_serial_line(line)
                if not data:
                    continue

                # Publish ultrasonic distances
                us_msg = Float32MultiArray()
                us_msg.data = [
                    float(data.get('U1', 0)),
                    float(data.get('U2', 0))
                ]
                self.ultrasonic_pub.publish(us_msg)

                # Publish line sensors
                line_msg = Int32MultiArray()
                line_msg.data = [
                    int(data.get('L1', 0)),
                    int(data.get('L2', 0)),
                    int(data.get('L3', 0))
                ]
                self.line_pub.publish(line_msg)

                # Publish IMU data
                imu_msg = Imu()
                imu_msg.angular_velocity.x = float(data.get('GX', 0))
                imu_msg.angular_velocity.y = float(data.get('GY', 0))
                imu_msg.angular_velocity.z = float(data.get('GZ', 0))
                imu_msg.linear_acceleration.x = float(data.get('AX', 0))
                imu_msg.linear_acceleration.y = float(data.get('AY', 0))
                imu_msg.linear_acceleration.z = float(data.get('AZ', 0))
                self.imu_pub.publish(imu_msg)

            except Exception as e:
                self.get_logger().warn(f"Sensor read error: {e}")

def main():
    rclpy.init()
    node = navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
