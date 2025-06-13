#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class IdentifierCommand(Node):
    def __init__(self):
        super().__init__('identifier_command_global')

        # Souscriptions pour chaque robot
        self.subscription_limo1 = self.create_subscription(
            String,
            '/limo1/identifier',
            lambda msg: self.handle_identifier_command(msg, 'limo1'),
            10
        )

        self.subscription_limo2 = self.create_subscription(
            String,
            '/limo2/identifier',
            lambda msg: self.handle_identifier_command(msg, 'limo2'),
            10
        )

        # Publishers pour les mouvements
        self.cmd_vel_pub_limo1 = self.create_publisher(Twist, '/limo1/cmd_vel', 10)
        self.cmd_vel_pub_limo2 = self.create_publisher(Twist, '/limo2/cmd_vel', 10)

        # Publisher commun pour les statuts
        self.status_publisher = self.create_publisher(String, '/identification_status', 10)

        # États des robots
        self.state = {'limo1': None, 'limo2': None}
        self.start_time = {'limo1': None, 'limo2': None}
        self.completed = {'limo1': False, 'limo2': False}
        self.stop_time = {'limo1': None, 'limo2': None}

        # Timer pour la boucle
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('✅ IdentifierCommand global node started.')

    def handle_identifier_command(self, msg, robot_ns):
        if msg.data == 'identify' and self.state[robot_ns] is None:
            self.get_logger().info(f'📩 IDENTIFY command received for {robot_ns}')
            self.state[robot_ns] = 'rotating'
            self.start_time[robot_ns] = time.time()
            self.completed[robot_ns] = False
            self.stop_time[robot_ns] = None

    def control_loop(self):
        now = time.time()
        for robot in ['limo1', 'limo2']:
            if self.state[robot] is None or self.start_time[robot] is None:
                continue

            elapsed = now - self.start_time[robot]
            twist = Twist()

            if elapsed < 4:
                twist.angular.z = 0.5 if int(elapsed) % 2 == 0 else -0.5
                pub = self.cmd_vel_pub_limo1 if robot == 'limo1' else self.cmd_vel_pub_limo2
                pub.publish(twist)
                self.get_logger().info(f'🔄 {robot} is turning {"LEFT" if twist.angular.z > 0 else "RIGHT"}')

            elif self.state[robot] == 'rotating':
                twist.angular.z = 0.0
                pub = self.cmd_vel_pub_limo1 if robot == 'limo1' else self.cmd_vel_pub_limo2
                pub.publish(twist)
                self.get_logger().info(f'🛑 {robot} stopped.')
                self.stop_time[robot] = now
                self.state[robot] = 'stopped'

            elif self.state[robot] == 'stopped' and now - self.stop_time[robot] >= 0.5:
                if not self.completed[robot]:
                    msg = String()
                    msg.data = "completed"
                    self.status_publisher.publish(msg)
                    self.get_logger().info(f'✅ Identification complete published: {msg.data}')
                    self.completed[robot] = True
                    self.state[robot] = None

def main():
    rclpy.init()
    node = IdentifierCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down IdentifierCommand Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
