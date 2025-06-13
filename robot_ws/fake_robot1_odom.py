import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point

class SimulatedOdomPublisher(Node):
    def __init__(self):
        super().__init__('simulated_robot1_odom')
        self.publisher = self.create_publisher(Pose, '/odom', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.step = 0

    def timer_callback(self):
        pose = Pose()
        pose.position = Point(x=self.step * 0.5, y=0.0, z=0.0)
        self.publisher.publish(pose)
        self.get_logger().info(f'📤 robot1 pose simulée: x = {pose.position.x:.2f} m')
        self.step += 1

def main():
    rclpy.init()
    node = SimulatedOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

