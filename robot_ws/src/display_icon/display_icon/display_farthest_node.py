# Fichier : display_farthest_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pygame

class DisplayFarthestNode(Node):
    def __init__(self):
        super().__init__('display_farthest_node')

        self.subscription = self.create_subscription(
            Bool,
            '/show_farthest_icon',
            self.callback,
            10
        )

        pygame.init()
        self.screen = pygame.display.set_mode((480, 320))  # taille typique pour écran LIMO
        pygame.display.set_caption('RF19 - Farthest Robot')
        self.font = pygame.font.Font(None, 60)
        self.is_displaying = False
        self.clear_screen()
        self.get_logger().info("DisplayFarthestNode started and ready.")

    def callback(self, msg: Bool):
        if msg.data and not self.is_displaying:
            self.display_message("🚀 Je suis le plus loin !")
            self.is_displaying = True
        elif not msg.data and self.is_displaying:
            self.clear_screen()
            self.is_displaying = False

    def display_message(self, text):
        self.screen.fill((0, 0, 0))
        message = self.font.render(text, True, (255, 255, 0))  # texte jaune
        text_rect = message.get_rect(center=(240, 160))
        self.screen.blit(message, text_rect)
        pygame.display.flip()

    def clear_screen(self):
        self.screen.fill((0, 0, 0))
        pygame.display.flip()

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DisplayFarthestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
