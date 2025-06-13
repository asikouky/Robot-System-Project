import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SimpleP2PTest(Node):
    def __init__(self):
        super().__init__('simple_p2p_test')
        self.pub = self.create_publisher(Bool, '/show_farthest_icon', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.toggle = False

    def timer_callback(self):
        self.toggle = not self.toggle
        self.get_logger().info(f"🟢 Publishing {self.toggle}")
        self.pub.publish(Bool(data=self.toggle))

def main():
    rclpy.init()
    node = SimpleP2PTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

