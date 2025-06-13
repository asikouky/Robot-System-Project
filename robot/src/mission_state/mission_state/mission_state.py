#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MissionState(Node):
    def __init__(self):
        super().__init__('mission_state')

        # Publisher pour envoyer l'état de la mission
        self.state_publisher = self.create_publisher(String, '/mission_status', 10)

        # Subscriber pour recevoir les commandes de mission
        self.command_subscription = self.create_subscription(
            String,
            '/mission_command',
            self.command_callback,
            10
        )

        # État initial du robot
        self.current_state = 'idle'
        self.get_logger().info("Mission State Node Started - Robot en attente")

    def command_callback(self, msg: String):
        """ Gère les commandes reçues et met à jour l'état du robot """
        command = msg.data.lower()
        new_state = self.current_state  # Garde l'état actuel par défaut

        if command == 'start':
            new_state = 'in_mission'
        elif command == 'stop':
            new_state = 'idle'  # Remet le robot en attente après un arrêt
        elif command == 'returning':
            new_state = 'returning'
        elif command == 'updating':
            new_state = 'updating'
        elif command == 'disconnected':
            new_state = 'disconnected'
        else:
            self.get_logger().warn(f"⚠️ Commande inconnue reçue : {command}")
            return

        # Mise à jour de l'état uniquement si un changement a eu lieu
        if new_state != self.current_state:
            self.current_state = new_state
            self.publish_state()

    def publish_state(self):
        """ Publie l'état actuel de la mission sur le topic /mission_status """
        msg = String()
        msg.data = self.current_state
        self.state_publisher.publish(msg)
        self.get_logger().info(f"📢 État de la mission mis à jour : {self.current_state}")

def main():
    rclpy.init()
    node = MissionState()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du Mission State Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
