#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AckermanLimo2Test(Node):
    def __init__(self):
        super().__init__('ackerman_limo2_test')
        self.vel_publisher = self.create_publisher(Twist, 'limo2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd = Twist()
        self.test_phase = 0
        self.counter = 0
        
        self.get_logger().info("==================================================")
        self.get_logger().info("LIMO2 ACKERMAN TEST STARTING")
        self.get_logger().info("==================================================")
        
        self.get_logger().info("Waiting for publishers to connect...")
        time.sleep(2.0)
        
        self.get_logger().info("TEST ACTION: Moving forward")
        self.cmd.linear.x = 1.5
        self.cmd.angular.z = 0.0

    def timer_callback(self):
        if self.test_phase == 0:
            if self.counter < 30:  # 3 seconds forward
                self.vel_publisher.publish(self.cmd)
                if self.counter % 10 == 0:
                    self.get_logger().info(f" Forward: linear.x={self.cmd.linear.x}, angular.z={self.cmd.angular.z}")
                self.counter += 1
            else:
                self.test_phase = 1
                self.counter = 0
                self.get_logger().info("TEST ACTION: Forward with left turn")
                self.cmd.linear.x = 1.0
                self.cmd.angular.z = 0.5  # gentle left turn
        
        elif self.test_phase == 1:
            if self.counter < 30:  # 3 seconds
                self.vel_publisher.publish(self.cmd)
                if self.counter % 10 == 0:
                    self.get_logger().info(f" Left curve: linear.x={self.cmd.linear.x}, angular.z={self.cmd.angular.z}")
                self.counter += 1
            else:
                self.test_phase = 2
                self.counter = 0
                self.get_logger().info("TEST ACTION: Forward with right turn")
                self.cmd.linear.x = 1.0
                self.cmd.angular.z = -0.5  # gentle right turn
        
        elif self.test_phase == 2:
            if self.counter < 30:  # 3 seconds
                self.vel_publisher.publish(self.cmd)
                if self.counter % 10 == 0:
                    self.get_logger().info(f" Right curve: linear.x={self.cmd.linear.x}, angular.z={self.cmd.angular.z}")
                self.counter += 1
            else:
                self.test_phase = 3
                self.counter = 0
                self.get_logger().info("TEST ACTION: Reverse with steering")
                self.cmd.linear.x = -0.7  # reverse
                self.cmd.angular.z = 0.3  # with some steering
        
        elif self.test_phase == 3:
            if self.counter < 30:  # 3 seconds
                self.vel_publisher.publish(self.cmd)
                if self.counter % 10 == 0:
                    self.get_logger().info(f" Reverse with steering: linear.x={self.cmd.linear.x}, angular.z={self.cmd.angular.z}")
                self.counter += 1
            else:
                self.test_phase = 4
                self.counter = 0
                self.get_logger().info("TEST ACTION: Stopping the robot")
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
        
        elif self.test_phase == 4:
            if self.counter < 10:
                self.vel_publisher.publish(self.cmd)
                if self.counter % 5 == 0:
                    self.get_logger().info(f" Stopping: linear.x={self.cmd.linear.x}, angular.z={self.cmd.angular.z}")
                self.counter += 1
            else:
                self.test_phase = 5
                self.get_logger().info("==================================================")
                self.get_logger().info("LIMO2 ACKERMAN TEST COMPLETE")
                self.get_logger().info("==================================================")
                self.destroy_node()
                rclpy.shutdown()

def main():
    rclpy.init()
    node = AckermanLimo2Test()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()