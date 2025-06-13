import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveLimo(Node):
    def __init__(self):
        super().__init__('move_limo')
        #self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_forward)
        self.get_logger().info("Limo Node Started")


    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Command Sent: Moving Forward!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveLimo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
