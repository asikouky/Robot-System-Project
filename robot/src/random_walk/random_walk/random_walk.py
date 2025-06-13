import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import math

class RandomWalkNode(Node):
    def __init__(self):
        super().__init__('random_walk')
        
        self.declare_parameter('enable_ackerman', True)
        self.enable_ackerman = self.get_parameter('enable_ackerman').value
        self.get_logger().info(f"Mode Ackerman activé: {self.enable_ackerman}")
        
        default_qos = QoSProfile(depth=10)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', default_qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, sensor_qos)
        self.command_sub = self.create_subscription(String, '/mission', self.command_callback, default_qos)
        self.timer = self.create_timer(0.2, self.move)
        
        self.move_cmd = Twist()
        self.obstacle = False
        self.avoid_mode = False
        self.active = False
        self.turn_direction = 1.0
        self.stuck_counter = 0
        self.reverse_mode = False
        self.reverse_timer = 0
        self.last_min_distance = 0.0
        self.turn_commitment_timer = 0
        self.committed_turn = False
        self.committed_direction = 0.0
        
        if self.enable_ackerman:
            self.min_forward_speed = 0.2
            self.normal_speed = 0.35
            self.max_turning_angle = 0.8
            self.reverse_speed = -0.25
            self.max_stuck_count = 20
            self.reverse_duration = 20
            self.turn_commitment_duration = 25
            self.obstacle_threshold = 1.0
        
        self.get_logger().info("🚶‍♂️ Nœud RandomWalk prêt et en attente de la commande 'start'")
    
    def command_callback(self, msg: String):
        if msg.data == "start":
            self.get_logger().info("✅ Commande DÉMARRER reçue → Activation du mouvement")
            self.active = True
        elif msg.data == "stop":
            self.get_logger().info("🛑 Commande ARRÊT reçue → Arrêt du robot")
            self.active = False
            self.move_cmd = Twist()
            self.cmd_pub.publish(self.move_cmd)
    
    def scan_callback(self, scan: LaserScan):
        if not self.active:
            return
        
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n = len(scan.ranges)
        
        angle_range_deg = 90
        angle_range_rad = math.radians(angle_range_deg)
        center_angle = 0.0
        center_index = int((center_angle - angle_min) / angle_inc)
        offset = int(angle_range_rad / angle_inc)
        start_index = max(0, center_index - offset)
        end_index = min(n, center_index + offset)
        
        front_ranges = [
            r for r in scan.ranges[start_index:end_index]
            if r > scan.range_min and r < scan.range_max
        ]
        
        if not front_ranges:
            self.get_logger().warn("❗ Aucun point valide détecté dans la zone frontale.")
            self.obstacle = False
            return
        
        min_distance = min(front_ranges)
        front_min_idx = front_ranges.index(min_distance)
        min_distance_idx = start_index + front_min_idx
        
        obstacle_threshold = 0.5
        if self.enable_ackerman:
            obstacle_threshold = self.obstacle_threshold
        
        self.obstacle = min_distance < obstacle_threshold
        
        if self.enable_ackerman and self.avoid_mode:
            is_very_close = min_distance < (obstacle_threshold * 0.5)
            distance_diff = abs(min_distance - self.last_min_distance)
            
            if is_very_close and distance_diff < 0.02:
                self.stuck_counter += 1
                if self.stuck_counter >= self.max_stuck_count and not self.reverse_mode:
                    self.get_logger().warn(f"⚠️ Le robot semble être bloqué à {min_distance:.2f}m, démarrage de la manœuvre de marche arrière")
                    self.reverse_mode = True
                    self.reverse_timer = 0
                    self.committed_turn = False
            else:
                self.stuck_counter = 0
                if not is_very_close and self.stuck_counter > 0:
                    self.get_logger().info("Pas assez proche de l'obstacle pour être considéré comme bloqué")
        
        self.last_min_distance = min_distance
        
        if self.enable_ackerman and self.obstacle and not self.committed_turn and not self.reverse_mode:
            obstacle_angle = angle_min + min_distance_idx * angle_inc
            self.turn_direction = -1.0 if obstacle_angle > 0 else 1.0
            
            if abs(obstacle_angle) < 0.1:
                self.turn_direction = 1.0
                self.get_logger().info(f"🧭 Obstacle directement devant, virage à GAUCHE par défaut")
            else:
                self.get_logger().info(f"🧭 Obstacle à un angle de {math.degrees(obstacle_angle):.1f}°, virage à {'GAUCHE' if self.turn_direction > 0 else 'DROITE'}")
            
            self.committed_turn = True
            self.committed_direction = self.turn_direction
            self.turn_commitment_timer = 0
            self.get_logger().info(f"🔄 Engagement dans un virage à {'GAUCHE' if self.turn_direction > 0 else 'DROITE'} pour les 5 prochaines secondes")
        
        if self.committed_turn:
            self.turn_commitment_timer += 1
            if self.turn_commitment_timer >= self.turn_commitment_duration:
                self.committed_turn = False
                self.get_logger().info("🔄 Période d'engagement du virage terminée, réévaluation de la direction")
            
        if self.avoid_mode and not self.obstacle and not self.reverse_mode and not self.committed_turn:
            self.get_logger().info("✅ Voie libre, reprise du mouvement")
            self.avoid_mode = False
        elif not self.avoid_mode and self.obstacle:
            self.get_logger().info(f"🚧 Obstacle détecté à {min_distance:.2f}m → Évitement")
            self.avoid_mode = True
    
    def move(self):
        if not self.active:
            return
        
        if self.enable_ackerman:
            self.move_ackerman()
        else:
            self.move_differential()
        
        self.cmd_pub.publish(self.move_cmd)
    
    def move_differential(self):
        if self.avoid_mode:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 2.0
        else:
            self.move_cmd.linear.x = 0.8
            self.move_cmd.angular.z = random.uniform(-0.4, 0.4)
    
    def move_ackerman(self):
        if self.reverse_mode:
            self.reverse_timer += 1
            
            if self.reverse_timer <= self.reverse_duration / 3:
                self.move_cmd.linear.x = self.reverse_speed * 1.2
                self.move_cmd.angular.z = 0.0
            elif self.reverse_timer <= 2 * self.reverse_duration / 3:
                self.move_cmd.linear.x = self.reverse_speed
                self.move_cmd.angular.z = self.max_turning_angle * self.turn_direction
            else:
                self.move_cmd.linear.x = self.reverse_speed * 0.8
                self.move_cmd.angular.z = self.max_turning_angle * self.turn_direction * 1.8
            
            if self.reverse_timer >= self.reverse_duration:
                self.get_logger().info("🔄 Manœuvre de marche arrière terminée, reprise du fonctionnement normal")
                self.reverse_mode = False
                self.stuck_counter = 0
                self.committed_turn = True
                self.committed_direction = self.turn_direction
                self.turn_commitment_timer = 0
                
        elif self.avoid_mode:
            if self.committed_turn:
                self.move_cmd.linear.x = self.min_forward_speed * 1.2
                self.move_cmd.angular.z = self.max_turning_angle * 1.2 * self.committed_direction
            else:
                self.move_cmd.linear.x = self.min_forward_speed
                self.move_cmd.angular.z = self.max_turning_angle * self.turn_direction
            
        else:
            self.move_cmd.linear.x = self.normal_speed
            self.move_cmd.angular.z = random.uniform(-0.25, 0.25) * self.max_turning_angle

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()