#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Mission(Node):
    def __init__(self):
        super().__init__('mission')

        # Subscriber pour écouter "start" ou "stop"
        self.subscription = self.create_subscription(
            String,
            '/mission',
            self.handle_move_command,
            10
        )

        # Publisher pour envoyer les commandes de mouvement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables de contrôle
        self.state = None  
        self.start_time = None
        self.movement_phase = 0  # 0 = Avancer, 1 = Reculer

        # Timer pour la boucle principale
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('🚀 Mission node started.')

    def handle_move_command(self, msg):
        """ Démarre ou arrête le cycle de mouvement en fonction du message reçu """
        if msg.data == 'start' and self.state is None:  # Permet de relancer la mission
            self.get_logger().info('✅ Commande START reçue, début du mouvement.')
            self.state = 'moving'
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.movement_phase = 0  # Commence en avançant

        elif msg.data == 'stop' and self.state == 'moving':
            self.get_logger().info('✅ Commande STOP reçue, arrêt du robot.')
            self.state = 'stopped'

            
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist)

            
            self.get_logger().info("🔄 Mission terminée, prête pour un nouveau départ.")
            self.state = None  # Ajout pour corriger le bug

    def control_loop(self):
        """ Boucle principale : alterne entre avancer et reculer """
        if self.state != 'moving' or self.start_time is None:
            return

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        twist = Twist()

        if (elapsed_time // 2) % 2 == 0:  # Alterner toutes les 2 secondes
            twist.linear.x = 0.3  # Avancer à 0.3 m/s
        else:
            twist.linear.x = -0.3  # Reculer à 0.3 m/s

        self.cmd_vel_publisher.publish(twist)

def main():
    rclpy.init()
    node = Mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Fermeture du nœud Mission...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
