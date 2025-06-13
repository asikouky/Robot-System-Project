import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import math

class RandomWalkNode(Node):
    def __init__(self):
        super().__init__('random_walk')

        # QoS Profiles
        default_qos = QoSProfile(depth=10)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', default_qos)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.command_sub = self.create_subscription(String, '/mission', self.command_callback, default_qos)

        # Timer (5 Hz)
        self.timer = self.create_timer(0.2, self.move)

        # State variables
        self.move_cmd = Twist()
        self.obstacle = False
        self.avoid_mode = False
        self.active = False  # Robot inactive until "start" command is received

        self.get_logger().info("🚶‍♂️ RandomWalkNode ready and waiting for the 'start' command")

    def command_callback(self, msg: String):
        if msg.data == "start":
            self.get_logger().info("✅ START command received → Activating movement")
            self.active = True
        elif msg.data == "stop":
            self.get_logger().info("🛑 STOP command received → Stopping the robot")
            self.active = False
            self.move_cmd = Twist()  # Immediate stop
            self.cmd_pub.publish(self.move_cmd)

    def scan_callback(self, scan: LaserScan):
        if not self.active:
            return

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n = len(scan.ranges)

        # Analyze field: ±40° (≈ 0.698 rad)
        angle_range_deg = 40
        angle_range_rad = math.radians(angle_range_deg)

        center_angle = 0.0  # directly in front
        center_index = int((center_angle - angle_min) / angle_inc)
        offset = int(angle_range_rad / angle_inc)

        start_index = max(0, center_index - offset)
        end_index = min(n, center_index + offset)

        front_ranges = [
            r for r in scan.ranges[start_index:end_index]
            if r > scan.range_min and r < scan.range_max
        ]

        if not front_ranges:
            self.get_logger().warn("❗ No valid points detected in the front area.")
            self.obstacle = False
            return

        min_distance = min(front_ranges)
        self.obstacle = min_distance < 0.5

        if self.avoid_mode and not self.obstacle:
            self.get_logger().info("✅ Path is clear, resuming movement")
            self.avoid_mode = False
        elif not self.avoid_mode and self.obstacle:
            self.get_logger().info(f"🚧 Obstacle detected at {min_distance:.2f}m → Avoiding")
            self.avoid_mode = True

    def move(self):
        if not self.active:
            return  # Inactive → don't publish anything

        if self.avoid_mode:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 1.0  # Rotate in place
        else:
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = random.uniform(-0.2, 0.2)

        self.cmd_pub.publish(self.move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

