#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RandomNavigator(Node):
    def __init__(self):
        super().__init__('random_navigator')
        # Publisher sur /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscriber sur /scan
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        # Paramètres
        self.obstacle_distance_threshold = 0.8  # distance minimale en m pour considérer qu'un obstacle est proche
        self.forward_speed = 0.2  # vitesse linéaire (m/s)
        self.turn_speed = 1.0  # vitesse angulaire (rad/s)
        self.state = "forward"  # état courant ("forward" ou "turn")
        self.turn_duration = 0.0  # durée de rotation restante
        self.timer = self.create_timer(0.1, self.timer_callback)  # timer de 100ms

    def scan_callback(self, msg: LaserScan):
        # Vérifie les distances dans un angle frontal (par exemple -30 à 30 degrés)
        # Indices approximatifs en fonction de l'angle de champ du laser
        min_angle = -0.5236  # -30 degrés en radians
        max_angle = 0.5236   # +30 degrés en radians
        ranges = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if min_angle <= angle <= max_angle:
                if distance < msg.range_max:
                    ranges.append(distance)
        if ranges:
            min_distance = min(ranges)
        else:
            min_distance = msg.range_max

        # Si un obstacle est proche et que l'on va de l'avant, passer en mode rotation
        if self.state == "forward" and min_distance < self.obstacle_distance_threshold:
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f} m, switching to turn')
            self.state = "turn"
            # Choisir une direction de rotation aléatoire (gauche ou droite)
            self.turn_direction = random.choice([-1, 1])
            # Définir une durée de rotation aléatoire entre 1 et 3 secondes
            self.turn_duration = random.uniform(1.0, 3.0)

    def timer_callback(self):
        twist = Twist()
        if self.state == "forward":
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
        elif self.state == "turn":
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed * self.turn_direction
            self.turn_duration -= 0.1
            if self.turn_duration <= 0:
                self.get_logger().info('Finished turning, going forward')
                self.state = "forward"
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = RandomNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
