import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 
import random

class LidarSim(Node):
    """
    Nodo de simulación de un LiDAR tipo TOF.
    Publica la distancia detectada en /lidar/front como un Float32.
    """
    def __init__(self):
        super().__init__('lidar_sim')
        self.publisher = self.create_publisher(Float32, 'lidar/front', 10)
        # Timer de publicación cada 0.2 segundos
        self.timer = self.create_timer(0.2, self.publish_distance)
        self.car_width_m = 0.25 
        self.distance_threshold_m = 0.28 

    def publish_distance(self):
        msg = Float32()

        # Simula una distancia detectada en metros (0.05 a 5.0 metros)
        # Se usa un número aleatorio para simular lecturas
        detected_distance = random.uniform(0.05, 5.0)

        # Publica la distancia simulada
        msg.data = detected_distance
        self.publisher.publish(msg)

        # Simula un escenario de obstáculo para el log
        status = "ok"
        if detected_distance <= self.distance_threshold_m + self.car_width_m:
            status = "obstaculo"
            
        self.get_logger().info(f"[LidarSim] Distancia: {detected_distance:.3f} m → {status}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()