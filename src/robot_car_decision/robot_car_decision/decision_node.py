import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DecisionNode(Node):
    """
    Nodo de decisión del carro.
    Combina Vision (color), Ultrasonic y Lidar.
    Toma decisiones periódicas basadas en la info de los sensores.
    """
    def __init__(self):
        super().__init__('decision_node')
        
        # Suscripciones
        self.create_subscription(String, 'vision/color', self.vision_cb, 10)
        self.create_subscription(String, 'ultrasonic/status', self.ultra_cb, 10)
        self.create_subscription(String, 'lidar/status', self.lidar_cb, 10)

        # Últimos valores recibidos
        self.last_color = None
        self.last_ultra_status = None
        self.last_lidar_status = None

        # Timer que ejecuta la decisión cada 0.5 segundos
        self.timer = self.create_timer(0.5, self.decide)

        # Configuración carro
        self.car_width_cm = 25

        self.get_logger().info("DecisionNode listo ✅")

    # Callbacks de los sensores
    def vision_cb(self, msg: String):
        self.last_color = msg.data.strip().lower()
        self.get_logger().info(f"[Visión] color: {self.last_color}")

    def ultra_cb(self, msg: String):
        self.last_ultra_status = msg.data.strip().lower()
        self.get_logger().info(f"[Ultrasonido] status: {self.last_ultra_status}")

    def lidar_cb(self, msg: String):
        self.last_lidar_status = msg.data.strip().lower()
        self.get_logger().info(f"[Lidar] status: {self.last_lidar_status}")

    # Lógica de decisión combinada
    def decide(self):
        if self.last_color is None or self.last_ultra_status is None or self.last_lidar_status is None:
            return  # Espera a recibir todos los datos

        # Si cualquiera de los sensores detecta obstáculo, detener
        if self.last_ultra_status == "obstaculo" or self.last_lidar_status == "obstaculo":
            print("⛔ Se detiene (obstáculo adelante)")
            return

        # Decisión basada en color
        if self.last_color == "red":
            print("➡️  Se mueve a la derecha")
            self.adjust_turn("right")
        elif self.last_color == "blue":
            print("⬅️  Se mueve a la izquierda")
            self.adjust_turn("left")
        elif self.last_color == "pink":
            print("🅿️  Parquea (detener)")
        else:
            print("⬆️  Sigue recto")

    def adjust_turn(self, direction):
        """
        Ajusta el giro considerando el ancho del carro.
        Por ahora solo simula el cálculo.
        """
        # Supongamos que para 25 cm de ancho, el radio mínimo de giro = 15 cm
        radius = 15
        print(f"[Giro] Calculando giro a la {direction} con radio mínimo {radius} cm")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
