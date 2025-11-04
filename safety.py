#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
from geometry_msgs.msg import Twist

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.get_logger().info("✅ Safety Node started")

        # === Parameters / tuning ===
        # value that indicates a line has been detected on a sensor (serial_bridge sends 0/1)
        self.line_trigger_value = 1
        self.correction_speed = 0.2     # lateral correction speed (m/s)
        self.preserve_forward = True    # keep forward velocity while correcting

        # === Internal state ===
        self.last_cmd_vel = Twist()
        self.line_data = []

        # === Waypoints (copy from motion_controller) ===
        self.waypoints = [
            (0.5, 0.0),
            (1.0, 0.5),
            (1.5, 0.0),
            (1.0, -0.5),
            (0.5, 0.0)
        ]
        self.current_waypoint = 0

        # === Per-waypoint monitor assignment ===
        # Each entry is a list of 1-based sensor indices to monitor for that waypoint.
        # Empty list means "no sensors monitored" (free movement).
        self.waypoint_monitors = [
            [1, 2],   # waypoint 0: monitor sensors L1 & L2
            [2, 3],   # waypoint 1: monitor sensors L2 & L3
            [],       # waypoint 2: free movement (no monitoring)
            [1],      # waypoint 3: monitor sensor L1 only
            [1, 2]    # waypoint 4: monitor sensors L1 & L2
        ]

        # Map sensor index -> corrective action ('left' or 'right')
        # Adjust to match physical layout: sensor 1 = L1, 2 = L2, 3 = L3
        self.sensor_action_map = {1: 'left', 2: 'right', 3: 'right'}

        # === ROS interfaces ===
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32MultiArray, '/line_sensors', self.line_callback, 10)
        self.create_subscription(Int32, '/waypoint_index', self.waypoint_index_callback, 10)

        self.safe_pub = self.create_publisher(Twist, '/safe_cmd_vel', 10)

        # periodic safety check (20 Hz)
        self.create_timer(0.05, self.safety_check)

    # -----------------------
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = msg

    def line_callback(self, msg: Int32MultiArray):
        # store as list of ints, expect values 0/1 from serial_bridge [L1,L2,L3]
        self.line_data = list(msg.data)

    def waypoint_index_callback(self, msg: Int32):
        idx = int(msg.data)
        if 0 <= idx < len(self.waypoints):
            self.current_waypoint = idx
            self.get_logger().info(f"➡️ Switched to waypoint {idx} {self.waypoints[idx]} monitors={self._get_monitors_for(idx)}")
        else:
            self.get_logger().warn(f"Waypoint index {idx} out of range (0..{len(self.waypoints)-1})")

    # -----------------------
    def _get_monitors_for(self, idx: int):
        if idx < len(self.waypoint_monitors):
            return self.waypoint_monitors[idx]
        return []

    def safety_check(self):
        safe_cmd = Twist()
        # start from last commanded velocity
        safe_cmd.linear.x = self.last_cmd_vel.linear.x
        safe_cmd.linear.y = self.last_cmd_vel.linear.y
        safe_cmd.angular.z = self.last_cmd_vel.angular.z

        monitors = self._get_monitors_for(self.current_waypoint)

        # If no monitors -> free movement
        if monitors:
            # iterate monitors in configured order; first triggered sensor wins
            for sensor_idx in monitors:
                arr_idx = sensor_idx - 1  # convert 1-based to 0-based
                if arr_idx < 0 or arr_idx >= len(self.line_data):
                    # sensor reading not available yet; skip
                    continue
                val = int(self.line_data[arr_idx])
                if val == self.line_trigger_value:
                    action = self.sensor_action_map.get(sensor_idx, 'right')
                    # set lateral correction until the sensor clears
                    if action == 'left':
                        safe_cmd.linear.y = -abs(self.correction_speed)
                    else:
                        safe_cmd.linear.y = abs(self.correction_speed)
                    if not self.preserve_forward:
                        safe_cmd.linear.x = 0.0
                    self.get_logger().warn(f"⚠️ Line sensor L{sensor_idx} triggered at waypoint {self.current_waypoint} — correcting {action}")
                    break  # apply only first applicable correction

        # publish safe command (will be equal to last_cmd_vel if no correction applied)
        self.safe_pub.publish(safe_cmd)

# -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Safety Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
