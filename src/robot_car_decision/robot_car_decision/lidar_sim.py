import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class LidarSim(Node):
    """
    Nodo de simulación de un LiDAR tipo TOF.
    Publica en /lidar/status "ok" o "obstaculo"
    según un umbral de distancia y un factor de ancho de carro.
    """
    def __init__(self):
        super().__init__('lidar_sim')
        self.publisher = self.create_publisher(String, 'lidar/status', 10)
        # Timer de publicación cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_status)
        self.car_width_cm = 25       # ancho del carro
        self.distance_threshold_cm = 20  # distancia mínima para considerar obstáculo

    def publish_status(self):
        msg = String()

        # Simula distancia detectada (0-100 cm)
        detected_distance = random.randint(5, 100)

        # Simula un obstáculo si la distancia es menor que el umbral + ancho del carro
        if detected_distance < self.distance_threshold_cm + self.car_width_cm:
            msg.data = "obstaculo"
        else:
            msg.data = "ok"

        self.publisher.publish(msg)
        self.get_logger().info(f"[LidarSim] Distancia: {detected_distance} cm → {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
