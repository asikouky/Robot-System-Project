#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from limo_msgs.msg import LimoStatus
import asyncio
import websockets
import json
import threading
from sensor_msgs.msg import BatteryState
from limo_msgs.msg import LimoStatus
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from enum import Enum

class EventType(Enum):
    MISSION_STATUS = "MISSION_STATUS"
    LOGS = "LOGS"
    BATTERY_LEVEL = "BATTERY_LEVEL"
    OBJECT_DETECTION = "OBJECT_DETECTION"
    CONNECTION_STATUS = "CONNECTION_STATUS"
    IDENTIFICATION = "IDENTIFICATION_STATUS"
    MAP = "MAP"
class WebSocketRosBridge(Node):
    def __init__(self):
        super().__init__('websocket_ros_bridge')

        self.topic_publishers = {}    
        self.websocket_client = None

        self.subscription = self.create_subscription(
            String,
            '/identification_status',
            self.identification_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(LimoStatus, '/limo_status', self.limo_status_callback, 10)

        self.get_logger().info("🚀 WebSocket-ROS Bridge Node Started")

    def get_publisher(self, topic_prefix: str, topic_name: str):
        full_topic = f"/{topic_prefix}/{topic_name}"
        if full_topic not in self.topic_publishers:
            if topic_name in ['mission', 'identifier']:
                pub = self.create_publisher(String, full_topic, 10)
            elif topic_name == 'p2p_toggle':
                pub = self.create_publisher(Bool, full_topic, 10)
            else:
                self.get_logger().warn(f"❓ Type de topic inconnu : {topic_name}")
                return None
            self.topic_publishers[full_topic] = pub
        return self.topic_publishers[full_topic]

    def identification_callback(self, msg: String):
        """ Callback exécutée lorsqu'un message est reçu sur /identification_status. """
        self.get_logger().info(f"📩 Message reçu sur /identification_status : {msg.data}")
        if self.websocket_client:
            try:
                ip = self.websocket_client.remote_address[0] if self.websocket_client.remote_address else "unknown"
                asyncio.run(self.websocket_client.send(json.dumps({"ip": ip, "status": msg.data, "eventType": EventType.IDENTIFICATION.value})))
                self.get_logger().info(f"✅ Message envoyé au WebSocket client [{ip}] : {msg.data}")
            except Exception as e:
                self.get_logger().error(f"⚠️ Erreur WebSocket : {e}")
    
    def limo_status_callback(self, msg: LimoStatus):
        """ Callback exécutée lorsqu'un message est reçu sur /limo_status """

        min_voltage = 8.4   # Tension quand la batterie est vide
        max_voltage = 12.6  # Tension quand la batterie est pleine
        battery_level = max(0, min(100, ((msg.battery_voltage - min_voltage) / (max_voltage - min_voltage)) * 100))

        self.last_battery_level = battery_level
        self.get_logger().info(f"⚡ Niveau de batterie : {battery_level:.1f}% ({msg.battery_voltage}V)")

        if battery_level < 30:
            self.get_logger().warn("⚠️ Batterie faible ! Activation du retour automatique à la base.")
            msg_mission = String()
            msg_mission.data = 'return_home'
            self.mission_publisher.publish(msg_mission)

        if self.websocket_client:
            try:
                ip = self.websocket_client.remote_address[0] if self.websocket_client.remote_address else "unknown"
                asyncio.run(self.websocket_client.send(json.dumps({"ip": ip, "battery": battery_level})))
                self.get_logger().info(f"✅ Niveau de batterie envoyé au WebSocket client [{ip}]: {battery_level:.1f}%")
            except Exception as e:
                self.get_logger().error(f"⚠️ Erreur WebSocket : {e}")
    def map_callback(self, msg: OccupancyGrid):
        """ Callback pour recevoir et transmettre le message de la carte. """
        self.get_logger().info("📡 Réception d'un message de carte.")

        # Conversion du message en un dictionnaire JSON
        map_data = {
            "info": {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin": {
                    "position": {
                        "x": msg.info.origin.position.x,
                        "y": msg.info.origin.position.y,
                        "z": msg.info.origin.position.z,
                    },
                    "orientation": {
                        "x": msg.info.origin.orientation.x,
                        "y": msg.info.origin.orientation.y,
                        "z": msg.info.origin.orientation.z,
                        "w": msg.info.origin.orientation.w,
                    }
                }
            },
            "data": list(msg.data)
        }
        
        self.get_logger().info(
            f"🗺️ Carte reçue : {msg.info.width}x{msg.info.height} (résolution {msg.info.resolution})"
        )
        
        if self.websocket_client:
            try:
                ip = self.websocket_client.remote_address[0] if self.websocket_client.remote_address else "unknown"
                payload = json.dumps({"map": map_data, "ip": ip})
                self.get_logger().info(f"< {payload}")
                asyncio.run(self.websocket_client.send(payload))
                self.get_logger().info(f"✅ Carte envoyée au client [{ip}]")
            except Exception as e:
                self.get_logger().error(f"⚠️ Erreur lors de l'envoi de la carte : {e}")
        else:
            self.get_logger().warn("⚠️ Aucun client WebSocket connecté pour recevoir la carte.")
    def odom_callback(self, msg: Odometry):
        # Extraction de la pose à partir du message Odometry
        pose_data = {
            "position": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z
            },
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w
            }
        }
        self.get_logger().info(f"📍 Pose via odom reçue : {pose_data}")
        if self.websocket_client:
            try:
                ip = self.websocket_client.remote_address[0] if self.websocket_client.remote_address else "unknown"
                asyncio.run(self.websocket_client.send(json.dumps({"pose": pose_data, "ip": ip})))
                self.get_logger().info(f"✅ Pose envoyée au client [{ip}]")
            except Exception as e:
                self.get_logger().error(f"⚠️ Erreur lors de l'envoi de la pose depuis /odom : {e}")
    
    async def handle_websocket(self, websocket):
        self.websocket_client = websocket
        ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
        self.get_logger().info(f"🌐 Client WebSocket connecté depuis [{ip}]")

        try:
            async for message in websocket:
                data = json.loads(message)
                command = data.get('command')
                topic_prefix = data.get('topic_prefix')

                if not topic_prefix:
                    self.get_logger().warn("⚠️ Aucune clé 'topic_prefix' fournie dans la commande.")
                    continue

                if command == 'identify':
                    self.get_logger().info(f'✅ Commande IDENTIFY reçue pour {topic_prefix}')
                    msg = String()
                    msg.data = 'identify'
                    self.identifier_publisher.publish(msg)

                    await websocket.send(json.dumps({"ip": ip, "status": "identification started", "eventType": EventType.IDENTIFICATION.value}))

                elif command == 'start':
                    self.get_logger().info(f'✅ Commande START reçue pour {topic_prefix}')
                    msg = String()
                    msg.data = 'start'
                    self.mission_publisher.publish(msg)

                    await websocket.send(json.dumps({"ip": ip, "status": "mission started"}))

                elif command == 'stop':
                    self.get_logger().info(f'✅ Commande STOP reçue pour {topic_prefix}')
                    msg = String()
                    msg.data = 'stop'
                    self.mission_publisher.publish(msg)

                    await websocket.send(json.dumps({"ip": ip, "status": "mission stopped"}))
                
                elif command == 'battery_status':
                    self.get_logger().info(f'📡 Demande du niveau de batterie par [{ip}]')
                    # Simuler une réponse pour l'instant
                    await websocket.send(json.dumps({"ip": ip, "battery": "fetching..."}))
                    await websocket.send(json.dumps({"ip": ip, "status": "mission stopped"}))
                
                elif command == 'battery_status':
                    self.get_logger().info(f'📡 Demande du niveau de batterie par [{ip}]')
                    # Simuler une réponse pour l'instant
                    await websocket.send(json.dumps({"ip": ip, "battery": "fetching..."}))

                elif command == 'P2P':
                    self.get_logger().info(f'✅ Commande P2P reçue pour {topic_prefix}')
                    msg = Bool()
                    msg.data = True
                    pub = self.get_publisher(topic_prefix, 'p2p_toggle')
                    if pub:
                        pub.publish(msg)
                        await websocket.send(json.dumps({"ip": ip, "status": "P2P mode activated"}))

                elif command == 'P2P_OFF':
                    self.get_logger().info(f'❎ Commande P2P_OFF reçue pour {topic_prefix}')
                    msg = Bool()
                    msg.data = False
                    pub = self.get_publisher(topic_prefix, 'p2p_toggle')
                    if pub:
                        pub.publish(msg)
                        await websocket.send(json.dumps({"ip": ip, "status": "P2P mode deactivated"}))

        except Exception as e:
            self.get_logger().error(f'⚠️ Erreur WebSocket : {e}')
        finally:
            self.websocket_client = None

# Serveur WebSocket
async def websocket_server(node):
    server = await websockets.serve(node.handle_websocket, "0.0.0.0", 8765)
    node.get_logger().info("🌍 WebSocket Server Running on ws://0.0.0.0:8765")
    await server.wait_closed()

def start_websocket_server(node):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(websocket_server(node))

def main():
    rclpy.init()
    node = WebSocketRosBridge()

    ws_thread = threading.Thread(target=start_websocket_server, args=(node,), daemon=True)
    ws_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Fermeture du WebSocket Bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
