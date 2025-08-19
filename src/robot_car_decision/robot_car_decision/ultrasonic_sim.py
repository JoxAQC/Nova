import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class UltrasonicSim(Node):
    """
    Nodo de simulación del sensor HC-SR04.
    Publica en /ultrasonic/status "ok" o "obstaculo".
    """
    def __init__(self):
        super().__init__('ultrasonic_sim')
        self.publisher = self.create_publisher(String, 'ultrasonic/status', 10)
        # Timer de publicación cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_status)
        self.car_width_cm = 25
        self.distance_threshold_cm = 15  # umbral del HC-SR04

    def publish_status(self):
        msg = String()
        # Simula distancia detectada (5-100 cm)
        detected_distance = random.randint(5, 100)
        # Obstáculo si la distancia < umbral + ancho carro
        if detected_distance < self.distance_threshold_cm + self.car_width_cm:
            msg.data = "obstaculo"
        else:
            msg.data = "ok"

        self.publisher.publish(msg)
        self.get_logger().info(f"[UltrasonicSim] Distancia: {detected_distance} cm → {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
