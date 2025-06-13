import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import socket
import json
import threading
import math
import os

MULTICAST_GROUP = '224.0.0.1'
MULTICAST_PORT = 5007

class P2PNode(Node):
    def __init__(self):
        super().__init__('p2p_node')

        # Lire les noms des robots depuis les variables d'environnement
        self.robot_name = os.getenv("ROBOT_NAME", "robot1")
        self.other_robot_name = os.getenv("OTHER_ROBOT_NAME", "robot2")

        self.start_pose = None
        self.local_distance = None
        self.other_distances = {}
        self.p2p_activated = True

        # Souscription à la position locale
        self.subscription_pose = self.create_subscription(
            Pose,
            '/odom',
            self.pose_callback,
            10
        )

        # Souscription à la commande P2P (bool venant du serveur via WebSocket)
        self.subscription_command = self.create_subscription(
            Bool,
            '/p2p_toggle',
            self.command_callback,
            10
        )

        # Publisher pour l'affichage sur écran tactile
        self.display_pub = self.create_publisher(Bool, '/show_farthest_icon', 10)

        # Lancer le thread d'écoute UDP
        self.listener_thread = threading.Thread(target=self.listen_udp, daemon=True)
        self.listener_thread.start()

        self.get_logger().info(f"P2PNode for {self.robot_name} started.")

    def command_callback(self, msg: Bool):
        self.p2p_activated = msg.data
        if self.p2p_activated:
            self.get_logger().info("P2P mode activated")
        else:
            self.get_logger().info("P2P mode deactivated")

    def pose_callback(self, msg: Pose):
        if not self.p2p_activated:
            return

        if self.start_pose is None:
            self.start_pose = msg.position
            self.get_logger().info(f"Start pose set: {self.start_pose}")
            return

        self.local_distance = self.euclidean_distance(self.start_pose, msg.position)
        self.send_distance()
        self.compare_distances()

    def send_distance(self):
        message = json.dumps({"name": self.robot_name, "distance": self.local_distance})
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        sock.sendto(message.encode(), (MULTICAST_GROUP, MULTICAST_PORT))

    def listen_udp(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', MULTICAST_PORT))
        mreq = socket.inet_aton(MULTICAST_GROUP) + socket.inet_aton('0.0.0.0')
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        while True:
            try:
                data, _ = sock.recvfrom(1024)
                message = json.loads(data.decode())
                name = message.get("name")
                dist = message.get("distance")
                if name != self.robot_name:
                    self.other_distances[name] = dist
            except Exception as e:
                self.get_logger().error(f"Error receiving UDP message: {e}")

    def compare_distances(self):
        if self.local_distance is None:
            return
        all_distances = self.other_distances.copy()
        all_distances[self.robot_name] = self.local_distance

        farthest = max(all_distances.items(), key=lambda x: x[1])
        is_farthest = (farthest[0] == self.robot_name)
        self.display_pub.publish(Bool(data=is_farthest))
        self.get_logger().info(f"I am {'farthest' if is_farthest else 'not farthest'}")

    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def main(args=None):
    rclpy.init(args=args)
    node = P2PNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
