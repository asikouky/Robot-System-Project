#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class IdentifierCommand(Node):
    def __init__(self):
        super().__init__('identifier_command')

        # Souscriptions
        self.subscription_r1 = self.create_subscription(
            String,
            '/robot1_104/identifier',
            lambda msg: self.handle_identifier_command(msg, 'robot1_104'),
            10
        )

        self.subscription_r2 = self.create_subscription(
            String,
            '/robot2_104/identifier',
            lambda msg: self.handle_identifier_command(msg, 'robot2_104'),
            10
        )

        # Publishers pour les commandes
        self.cmd_vel_pub_r1 = self.create_publisher(Twist, '/robot1_104/cmd_vel', 10)
        self.cmd_vel_pub_r2 = self.create_publisher(Twist, '/robot2_104/cmd_vel', 10)

        # Publisher pour le statut global
        self.status_publisher = self.create_publisher(String, '/identification_status', 10)

        # États par robot
        self.state = {'robot1_104': None, 'robot2_104': None}
        self.start_time = {'robot1_104': None, 'robot2_104': None}
        self.completed = {'robot1_104': False, 'robot2_104': False}
        self.stop_time = {'robot1_104': None, 'robot2_104': None}

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('✅ IdentifierCommand node started (robot1_104 / robot2_104).')

    def handle_identifier_command(self, msg, robot_ns):
        if msg.data == 'identify' and self.state[robot_ns] is None:
            self.get_logger().info(f'📩 IDENTIFY received for {robot_ns}')
            self.state[robot_ns] = 'rotating'
            self.start_time[robot_ns] = time.time()
            self.completed[robot_ns] = False
            self.stop_time[robot_ns] = None

    def control_loop(self):
        now = time.time()
        for robot in ['robot1_104', 'robot2_104']:
            if self.state[robot] is None:
                continue

            elapsed = now - self.start_time[robot]
            twist = Twist()

            if elapsed < 4:
                twist.angular.z = 0.5 if int(elapsed) % 2 == 0 else -0.5
                pub = self.cmd_vel_pub_r1 if robot == 'robot1_104' else self.cmd_vel_pub_r2
                pub.publish(twist)
                self.get_logger().info(f'🔄 {robot} turning {"LEFT" if twist.angular.z > 0 else "RIGHT"}')

            elif self.state[robot] == 'rotating':
                twist.angular.z = 0.0
                pub = self.cmd_vel_pub_r1 if robot == 'robot1_104' else self.cmd_vel_pub_r2
                pub.publish(twist)
                self.get_logger().info(f'🛑 {robot} stopped.')
                self.stop_time[robot] = now
                self.state[robot] = 'stopped'

            elif self.state[robot] == 'stopped' and now - self.stop_time[robot] >= 0.5:
                if not self.completed[robot]:
                    status = String()
                    status.data = f"{robot}: completed"
                    self.status_publisher.publish(status)
                    self.get_logger().info(f'✅ {robot} identification completed.')
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
