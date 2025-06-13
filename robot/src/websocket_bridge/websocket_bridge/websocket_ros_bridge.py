#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import json
import threading
from common_interfaces.event_types import EventType
import base64
import os
import yaml
from PIL import Image
from .map_sender import MapSender



class WebSocketRosBridge(Node):
    def __init__(self):
        super().__init__('websocket_ros_bridge')

        # Publisher pour commander l'identification du robot
        self.identifier_publisher = self.create_publisher(String, '/identifier', 10)

        # Publisher pour commander le mouvement du robot (start/stop)
        self.mission_publisher = self.create_publisher(String, '/mission', 10)

        # Subscriber pour écouter la fin d'identification
        self.subscription = self.create_subscription(
            String,
            '/identification_status',
            self.identification_callback,
            10
        )

        # Variable pour stocker la référence au subscriber du scan laser
        self.laser_subscription = None
        # Flag pour suivre l'état du scan laser
        self.laser_scan_active = False
        
        # Activer le scan par défaut
        self.enable_laser_scan()

        # Dictionnaire des topics ROS et leurs types d'événements associés
        self.topic_event_map = {
            "/mission_status": EventType.MISSION_STATUS,
            "/logs": EventType.LOGS,
            "/battery_level": EventType.BATTERY_LEVEL,
            "/object_detection": EventType.OBJECT_DETECTION,
            "/connection_status": EventType.CONNECTION_STATUS
        }

        # Création dynamique des abonnements aux topics ROS
        for topic, event_type in self.topic_event_map.items():
            self.create_subscription(String, topic, lambda msg, et=event_type: self.generic_callback(msg, et), 10)

        self.get_logger().info("🚀 WebSocket-ROS Bridge Node Started")

        self.websocket_client = None  # Stocke le client WebSocket actif

        self.map_sender = MapSender(self.get_websocket_client)


    def get_websocket_client(self):
        return self.websocket_client
  
    def enable_laser_scan(self):
        """Active la souscription au topic de scan laser."""
        if not self.laser_scan_active:
            # Créer la souscription uniquement si elle n'existe pas déjà
            self.laser_subscription = self.create_subscription(
                String,
                'limo1/processed_scan',
                lambda msg: self.debug_scan_callback(msg, 'limo1'),
                10
            )
            self.laser_subscription = self.create_subscription(
                String,
                'limo2/processed_scan',
                lambda msg: self.debug_scan_callback(msg, 'limo2'),
                10
            )
            self.laser_scan_active = True
            self.get_logger().info("📡 Laser scan subscription enabled")
            
    def disable_laser_scan(self):
        """Désactive la souscription au topic de scan laser."""
        if self.laser_scan_active and self.laser_subscription:
            # Destroy the subscription
            self.destroy_subscription(self.laser_subscription)
            self.laser_subscription = None
            self.laser_scan_active = False
            self.get_logger().info("📡 Laser scan subscription disabled")

    def identification_callback(self, msg: String):
        """ Callback exécutée lorsqu'un message est reçu sur /identification_status. """
        self.get_logger().info(f"📩 Message reçu sur /identification_status : {msg.data}")

        
        if self.websocket_client:
            try:
                ip = self.websocket_client.remote_address[0] if self.websocket_client.remote_address else "unknown"

                message = {
                    "eventType": EventType.IDENTIFICATION.value,
                    "data": {
                        "status": msg.data,
                        "ip": ip
                    }
                }

                asyncio.run(self.websocket_client.send(json.dumps(message)))
                self.get_logger().info(f"✅ Message IDENTIFICATION envoyé au WebSocket client [{ip}] : {msg.data}")
            except Exception as e:
                self.get_logger().error(f"⚠️ Erreur WebSocket : {e}")


    def send_to_websocket(self, event_type: EventType, data: str):
        """ Envoie un message formaté au WebSocket client """
        if self.websocket_client:
            try:
                message = json.dumps({"eventType": event_type.value, "data": data})
                asyncio.run(self.websocket_client.send(message))
                self.get_logger().info(f"✅ Message envoyé au WebSocket client : {message}")
            except Exception as e:
                self.get_logger().error(f"⚠️ Erreur WebSocket : {e}")

    def generic_callback(self, msg: String, event_type: EventType):
        """ Callback générique pour tous les topics """
        self.send_to_websocket(event_type, msg.data)



    def debug_scan_callback(self, msg: String, robot: String):
        """ Callback exécutée lorsqu'un message est reçu sur /processed_scan. """
        self.get_logger().debug(f"📡 LaserScan data received")
        
        
        if self.websocket_client:
            try:
                ip = self.websocket_client.remote_address[0] if self.websocket_client.remote_address else "unknown"

                # The data is already in JSON format, parse it to add message type
                laser_data = json.loads(msg.data)
                
                # Handle Infinity values before sending
                for key, value in laser_data.items():
                    if value == float('inf') or value == float('-inf'):
                        laser_data[key] = "Infinity"

                # Add message type for client differentiation
                message = {"eventType": EventType.LOGS.value,
                           "type": "debug_laser_data", 
                           "ip": ip,
                           "robot": robot,
                           "data": laser_data}
                
                asyncio.run(self.websocket_client.send(json.dumps(message)))
                self.get_logger().debug(f"✅ Laser data sent to WebSocket client [{ip}]")
            except Exception as e:
                self.get_logger().error(f"⚠️ Erreur WebSocket (laser data) : {e}")

    def get_publisher(self, namespace: str, topic_name: str):
        """Crée dynamiquement un publisher sur /<namespace>/<topic_name>."""
        full_topic = f'/{namespace}/{topic_name}'
        return self.create_publisher(String, full_topic, 10)

    
    async def handle_websocket(self, websocket):
        """ Gère la connexion WebSocket avec un seul client. """
        self.websocket_client = websocket  # Stocke le client actif
        ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
        self.get_logger().info(f"🌐 Client WebSocket connecté depuis [{ip}]")

        try:
            async for message in websocket:
                data = json.loads(message)
                command = data.get('command')
                target_robot = data.get('robot')  # ex: "limo" ou "limo2"
                
                if command == 'identify':
                    if target_robot not in ['limo1', 'limo2']:
                        self.get_logger().warn(f"⚠️ Robot cible non spécifié ou invalide. Reçu : {target_robot}")
                        continue

                    pub = self.get_publisher(target_robot, 'identifier')
                    msg = String()
                    msg.data = 'identify'
                    pub.publish(msg)

                    self.get_logger().info(f'📤 Commande IDENTIFY publiée sur /{target_robot}/identifier')

                    await websocket.send(json.dumps({
                        "eventType": EventType.IDENTIFICATION.value,
                        "data": {
                            "ip": ip,
                            "status": "identification started",
                            "robot": target_robot
                        }
                    }))


                elif command == 'start':
                    self.get_logger().info(f'✅ Commande START reçue depuis [{ip}], publication sur /mission')
                    msg = String()
                    msg.data = 'start'
                    self.mission_publisher.publish(msg)
                    
                    # Enable laser scan when starting mission
                    self.enable_laser_scan()

                    await websocket.send(json.dumps({
                        "eventType": EventType.MISSION_STATUS.value,
                        "data": {
                            "ip": ip,
                            "status": "in_mission"
                        }
                    }))

                    self.map_sender.start_sending(interval_seconds=2)  # toutes les 2 secondes


                elif command == 'stop':
                    self.get_logger().info(f'✅ Commande STOP reçue depuis [{ip}], publication sur /mission')
                    msg = String()
                    msg.data = 'stop'
                    self.mission_publisher.publish(msg)
                    
                    # Disable laser scan when stopping mission
                    self.disable_laser_scan()
                    self.map_sender.stop_sending()

                    

                    await websocket.send(json.dumps({
                        "eventType": EventType.MISSION_STATUS.value,
                        "data": {
                            "ip": ip,
                            "status": "idle"
                        }
                    }))

                    await self.map_sender.send_map_update()


        except Exception as e:
            self.get_logger().error(f'⚠️ Erreur WebSocket : {e}')
        finally:
            self.websocket_client = None  # Déconnexion du client

async def websocket_server(node):
    """ Démarre un serveur WebSocket en parallèle. """
    server = await websockets.serve(node.handle_websocket, "0.0.0.0", 8765)
    node.get_logger().info("🌍 WebSocket Server Running on ws://0.0.0.0:8765")

    await server.wait_closed()  # Attend la fermeture

def start_websocket_server(node):
    """ Lance le serveur WebSocket dans un thread proprement. """
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(websocket_server(node))

def main():
    rclpy.init()
    node = WebSocketRosBridge()

    #  Démarrer le serveur WebSocket dans un thread propre avec asyncio
    ws_thread = threading.Thread(target=start_websocket_server, args=(node,), daemon=True)
    ws_thread.start()

    #  Faire tourner le nœud ROS normalement
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Fermeture du WebSocket Bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
