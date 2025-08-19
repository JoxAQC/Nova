import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random, time

class VisionSim(Node):
    def __init__(self):
        super().__init__('vision_sim')
        self.publisher = self.create_publisher(String, 'vision/color', 10)
        self.timer = self.create_timer(2.0, self.publish_color)
        self.colors = ["red", "blue", "pink", "none"]

    def publish_color(self):
        msg = String()
        msg.data = random.choice(self.colors)
        self.publisher.publish(msg)
        self.get_logger().info(f"[VisionSim] Publicado: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
