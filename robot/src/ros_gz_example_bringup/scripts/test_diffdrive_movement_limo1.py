#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Limo1Test(Node):
    def __init__(self):
        super().__init__('limo1_test')
        self.publisher = self.create_publisher(Twist, 'limo1/cmd_vel', 10)
        
        self.get_logger().info("==================================================")
        self.get_logger().info("LIMO1 TEST STARTING")
        self.get_logger().info("==================================================")
        
        self.get_logger().info("Waiting for publisher to connect...")
        time.sleep(2.0)
        
        self.msg = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.phase = 0
        self.count = 0
        
        self.get_logger().info("TEST ACTION: Moving forward")
        self.msg.linear.x = 1.5
        self.msg.angular.z = 0.0

    def timer_callback(self):
        if self.phase == 0:
            if self.count < 30:  # 3 seconds forward
                self.publisher.publish(self.msg)
                if self.count % 10 == 0:
                    self.get_logger().info(f" Forward: linear.x={self.msg.linear.x}, angular.z={self.msg.angular.z}")
                self.count += 1
            else:
                self.phase = 1
                self.count = 0
                self.get_logger().info("TEST ACTION: Left turn")
                self.msg.linear.x = 0.0
                self.msg.angular.z = 1.0  # left turn
        
        elif self.phase == 1:
            if self.count < 30:  # 3 seconds turn
                self.publisher.publish(self.msg)
                if self.count % 10 == 0:
                    self.get_logger().info(f" Left turn: linear.x={self.msg.linear.x}, angular.z={self.msg.angular.z}")
                self.count += 1
            else:
                self.phase = 2
                self.count = 0
                self.get_logger().info("TEST ACTION: Right turn")
                self.msg.linear.x = 0.0
                self.msg.angular.z = -1.0  # right turn
        
        elif self.phase == 2:
            if self.count < 30:  # 3 seconds turn
                self.publisher.publish(self.msg)
                if self.count % 10 == 0:
                    self.get_logger().info(f" Right turn: linear.x={self.msg.linear.x}, angular.z={self.msg.angular.z}")
                self.count += 1
            else:
                self.phase = 3
                self.count = 0
                self.get_logger().info("TEST ACTION: Reverse")
                self.msg.linear.x = -0.7  # reverse
                self.msg.angular.z = 0.0
        
        elif self.phase == 3:
            if self.count < 30:  # 3 seconds reverse
                self.publisher.publish(self.msg)
                if self.count % 10 == 0:
                    self.get_logger().info(f" Reverse: linear.x={self.msg.linear.x}, angular.z={self.msg.angular.z}")
                self.count += 1
            else:
                self.phase = 4
                self.count = 0
                self.get_logger().info("TEST ACTION: Stopping the robot")
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
        
        elif self.phase == 4:
            if self.count < 10:
                self.publisher.publish(self.msg)
                if self.count % 5 == 0:
                    self.get_logger().info(f" Stopping: linear.x={self.msg.linear.x}, angular.z={self.msg.angular.z}")
                self.count += 1
            else:
                self.phase = 5
                self.get_logger().info("==================================================")
                self.get_logger().info("LIMO1 TEST COMPLETE")
                self.get_logger().info("==================================================")
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()

def main():
    rclpy.init()
    node = Limo1Test()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()