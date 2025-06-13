import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import threading
import math

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        self.sub_map1 = self.create_subscription(
            OccupancyGrid, '/limo1/map', self.callback_map1, 10)

        self.sub_map2 = self.create_subscription(
            OccupancyGrid, '/limo2/map', self.callback_map2, 10)

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub_merged_map = self.create_publisher(OccupancyGrid, '/merged_map', qos)

        self.map1 = None
        self.map2 = None
        self.lock = threading.Lock()

        # Ajout : garder une trace de la dernière carte publiée
        self.last_published_map = None
        self.last_pub_time = self.get_clock().now()
        self.pub_interval_sec = 5.0  # Publier au moins toutes les 5 secondes

    def callback_map1(self, msg):
        with self.lock:
            self.map1 = msg
            self.try_publish()

    def callback_map2(self, msg):
        with self.lock:
            self.map2 = msg
            self.try_publish()

    def try_publish(self):
        if self.map1 is None or self.map2 is None:
            return

        res1 = self.map1.info.resolution
        res2 = self.map2.info.resolution

        if res1 != res2:
            self.get_logger().warn("Les résolutions des cartes diffèrent !")
            return

        res = res1

        def map_bounds(map_msg):
            ox = map_msg.info.origin.position.x
            oy = map_msg.info.origin.position.y
            w = map_msg.info.width
            h = map_msg.info.height
            return ox, oy, ox + w * res, oy + h * res

        # Obtenir les coins min/max du canvas global
        min_x = min(map_bounds(self.map1)[0], map_bounds(self.map2)[0])
        min_y = min(map_bounds(self.map1)[1], map_bounds(self.map2)[1])
        max_x = max(map_bounds(self.map1)[2], map_bounds(self.map2)[2])
        max_y = max(map_bounds(self.map1)[3], map_bounds(self.map2)[3])

        merged_w = math.ceil((max_x - min_x) / res)
        merged_h = math.ceil((max_y - min_y) / res)

        merged_array = -1 * np.ones((merged_h, merged_w), dtype=np.int8)

        def insert_map(map_msg):
            ox = map_msg.info.origin.position.x
            oy = map_msg.info.origin.position.y
            w = map_msg.info.width
            h = map_msg.info.height
            offset_x = int((ox - min_x) / res)
            offset_y = int((oy - min_y) / res)
            arr = np.array(map_msg.data, dtype=np.int8).reshape((h, w))
            merged_array[offset_y:offset_y + h, offset_x:offset_x + w] = np.maximum(
                merged_array[offset_y:offset_y + h, offset_x:offset_x + w], arr)

        insert_map(self.map1)
        insert_map(self.map2)

        # Vérifier si la carte a changé ou si temps écoulé
        should_publish = False
        now = self.get_clock().now()
        elapsed = (now - self.last_pub_time).nanoseconds * 1e-9

        if self.last_published_map is None:
            should_publish = True
        else:
            has_changed = not np.array_equal(merged_array, self.last_published_map)
            if has_changed or elapsed >= self.pub_interval_sec:
                should_publish = True

        if not should_publish:
            return

        self.last_published_map = merged_array.copy()
        self.last_pub_time = now

        merged_msg = OccupancyGrid()
        merged_msg.header.frame_id = 'map'
        merged_msg.header.stamp = now.to_msg()
        merged_msg.info.resolution = res
        merged_msg.info.width = merged_w
        merged_msg.info.height = merged_h
        merged_msg.info.origin.position.x = min_x
        merged_msg.info.origin.position.y = min_y
        merged_msg.info.origin.position.z = 0.0
        merged_msg.info.origin.orientation.w = 1.0
        merged_msg.data = merged_array.flatten().tolist()

        self.pub_merged_map.publish(merged_msg)
        #self.get_logger().info(f"Carte fusionnée publiée ({merged_w}x{merged_h})")


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
