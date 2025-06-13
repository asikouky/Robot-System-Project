import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import time
from datetime import datetime
import json


class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN) # Remove this line to enable INFO logs

        self.last_log_time = time.time()
        self.log_interval = 1.0 # interval for 1Hz sampling

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            String,
            'processed_scan',
            10)
        

    def listener_callback(self, msg):
        current_time = time.time()

        # We are aiming for 1 Hz sampling frequency. Make sure 1s has passed since last sample
        if current_time - self.last_log_time < self.log_interval:
            return
        
        self.last_log_time = current_time

        stamp = msg.header.stamp
        timestamp_sec = stamp.sec + stamp.nanosec / 1e9
        human_time = datetime.fromtimestamp(timestamp_sec).strftime('%H:%M:%S.%f')[:-3]

        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        if not valid_ranges:
            self.get_logger().warn('No valid laser readings')
            return
        
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]

        # Safety information
        min_distance = min(valid_ranges)
        min_index = msg.ranges.index(min_distance)
        min_angle_deg = np.degrees(msg.angle_min + min_index * msg.angle_increment)
        
        safety_threshold = 0.5 # 50 cm
        close_obstacles = sum(1 for r in valid_ranges if r < safety_threshold)

        # Directional sector information (front, back, left, right)
        front_index = [i for i, a in enumerate(angles) if -np.pi / 4 <= a <= np.pi / 4]
        left_index = [i for i, a in enumerate(angles) if np.pi / 4 <= a <= 3 * np.pi / 4]
        right_index = [i for i, a in enumerate(angles) if -3 * np.pi / 4 <= a <= -np.pi / 4]
        back_index = [i for i, a in enumerate(angles) if abs(a) >= 3 * np.pi / 4]

        front_min = min([msg.ranges[i] for i in front_index if msg.range_min <= msg.ranges[i] <= msg.range_max], default=float('inf'))
        left_min = min([msg.ranges[i] for i in left_index if msg.range_min <= msg.ranges[i] <= msg.range_max], default=float('inf'))
        right_min = min([msg.ranges[i] for i in right_index if msg.range_min <= msg.ranges[i] <= msg.range_max], default=float('inf'))
        back_min = min([msg.ranges[i] for i in back_index if msg.range_min <= msg.ranges[i] <= msg.range_max], default=float('inf'))

        # Environmental information
        average_distance = sum(valid_ranges) / len(valid_ranges)
        distance_variance = np.var(valid_ranges) if len(valid_ranges) > 1 else 0

        # Processed LaserScan message
        processed_data = {
            "timestamp": human_time,
            "min_distance": round(min_distance, 2),
            "min_angle_deg": round(min_angle_deg, 0),
            "close_obstacles": close_obstacles,
            "front_min": round(front_min, 2),
            "left_min": round(left_min, 2),
            "right_min": round(right_min, 2),
            "back_min": round(back_min, 2),
            "average_distance": round(average_distance, 2),
            "distance_variance": round(distance_variance, 2)
        }

        # Publish as JSON string
        json_str = json.dumps(processed_data)
        string_msg = String()
        string_msg.data = json_str
        self.publisher.publish(string_msg)

        self.get_logger().info(
            f"SCAN_LOG: "
            f"t={human_time} | "
            f"min_dist={min_distance:.2f}m@{min_angle_deg:.0f}° | "
            f"obstacles(<{safety_threshold}m)={close_obstacles} | "
            f"F/L/R/B={front_min:.2f}/{left_min:.2f}/{right_min:.2f}/{back_min:.2f}m | "
            f"avg={average_distance:.2f}m var={distance_variance:.2f}"
        )



def main(args=None):
    rclpy.init(args=args)

    laser_scan_processor = LaserScanProcessor()
    rclpy.spin(laser_scan_processor)

    laser_scan_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()