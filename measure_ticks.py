#!/usr/bin/env python3
# measure_ticks.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

class TickMeasurer(Node):
    def __init__(self, wheel_index=0, rotations=5):
        super().__init__('tick_measurer')
        self.sub = self.create_subscription(Int32MultiArray, '/encoders', self.cb, 10)
        self.last = None
        self.count = 0
        self.wheel_index = wheel_index
        self.rotations = rotations
        self.start_time = None
        self.start_val = None
        self.get_logger().info("Tick measurer ready. When ready, press Enter to start measurement.")
        input("Press Enter to start: ")
        self.get_logger().info(f"Start rotating wheel {rotations} full turns now (wheel index {wheel_index}).")
        self.start_time = time.time()

    def cb(self, msg: Int32MultiArray):
        val = int(msg.data[self.wheel_index])
        if self.start_val is None:
            self.start_val = val
            self.last = val
            return
        # accumulate difference (handles possible wrap / negative)
        diff = val - self.last
        self.count += diff
        self.last = val

        # If we've seen total absolute count >= rotations * (estimated per-rotation)
        # we can't reliably know rotations, so instead let user stop with Enter
        # We print intermediate progress
        if int(time.time() - self.start_time) % 2 == 0:
            self.get_logger().info(f"Current raw value: {val}, accumulated Δ: {self.count}")

def main():
    rclpy.init()
    node = TickMeasurer(wheel_index=0, rotations=5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        print("Measurement stopped. Note start_val and last_val from logs and compute ticks/rotation manually.")
