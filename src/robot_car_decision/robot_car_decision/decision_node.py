import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # ====== Parámetros del vehículo / pista (ajústalos a tu coche real) ======
        self.declare_parameter('wheelbase', 0.22)          # L (m)
        self.declare_parameter('track', 0.18)              # T (m) distancia entre ruedas delanteras
        self.declare_parameter('car_width', 0.25)          # W (m) ancho total del coche
        self.declare_parameter('max_steer_deg', 35.0)      # tope mecánico de los servos
        self.declare_parameter('lane_width', 1.00)         # m (Desafío de Obstáculos)
        self.declare_parameter('pillar_half', 0.025)       # 50 mm / 2
        self.declare_parameter('safety_margin', 0.03)      # margen extra contra muro/pilar (m)
        self.declare_parameter('lookahead', 0.50)          # Ld (m) Pure Pursuit
        self.declare_parameter('stop_dist', 0.28)          # m (½ coche + margen) para STOP
        self.declare_parameter('base_speed', 0.65)         # 0..1 PWM proporcional
        self.declare_parameter('min_speed', 0.35)
        self.declare_parameter('fresh_t', 0.8)             # s, ventana de frescura de sensores

        # Lee parámetros
        self.L = float(self.get_parameter('wheelbase').value)
        self.T = float(self.get_parameter('track').value)
        self.W = float(self.get_parameter('car_width').value)
        self.max_deg = float(self.get_parameter('max_steer_deg').value)
        self.lane_w = float(self.get_parameter('lane_width').value)
        self.pillar_half = float(self.get_parameter('pillar_half').value)
        self.margin = float(self.get_parameter('safety_margin').value)
        self.Ld = float(self.get_parameter('lookahead').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.base_speed = float(self.get_parameter('base_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.fresh_t = float(self.get_parameter('fresh_t').value)

        # ====== Subscriptores ======
        self.last_color = None; self.t_color = 0.0
        self.last_ultra = None; self.t_ultra = 0.0
        self.last_lidar = None; self.t_lidar = 0.0

        self.create_subscription(String, 'vision/color', self.vision_cb, 10)
        self.create_subscription(String, 'ultrasonic/status', self.ultra_cb, 10)
        self.create_subscription(Float32, 'lidar/front', self.lidar_cb, 10)

        # ====== Publicadores (motores y servos) ======
        self.pub_lm = self.create_publisher(Float32, 'cmd/left_motor', 10)
        self.pub_rm = self.create_publisher(Float32, 'cmd/right_motor', 10)
        self.pub_ls = self.create_publisher(Float32, 'cmd/left_servo', 10)   # grados
        self.pub_rs = self.create_publisher(Float32, 'cmd/right_servo', 10)  # grados
        self.pub_action = self.create_publisher(String, 'cmd/action', 10)

        # Timer de decisión
        self.timer = self.create_timer(0.05, self.decide)
        self.get_logger().info("DecisionNode listo ✅")

    # ====== Callbacks ======
    def vision_cb(self, msg: String):
        self.last_color = msg.data.strip().lower()
        self.t_color = time.time()
        self.get_logger().info(f"[Visión] color: {self.last_color}")

    def ultra_cb(self, msg: String):
        self.last_ultra = msg.data.strip().lower()
        self.t_ultra = time.time()
        self.get_logger().info(f"[Ultrasonido] status: {self.last_ultra}")

    def lidar_cb(self, msg: Float32):
        self.last_lidar = float(msg.data)
        self.t_lidar = time.time()
        self.get_logger().info(f"[Lidar] d={self.last_lidar:.3f} m")

    # ====== Utilidades de control ======
    def ackermann_from_center_angle(self, delta_deg):
        """
        Convierte un ángulo "central" (aprox. del eje virtual) a ángulos de servo
        internos/externos usando Ackermann: tan(delta_in)=L/(R - T/2), tan(delta_out)=L/(R + T/2)
        donde R = L / tan(delta_center).
        """
        delta_deg = clamp(delta_deg, -self.max_deg, self.max_deg)
        if abs(delta_deg) < 1e-3:
            return 0.0, 0.0

        delta = math.radians(delta_deg)
        R = self.L / math.tan(delta)
        # Signo del giro: delta>0 gira a la izquierda; ajustamos “inner/outer”
        left_is_inner = delta_deg > 0.0
        try:
            delta_in = math.degrees(math.atan(self.L / (abs(R) - self.T/2)))
            delta_out = math.degrees(math.atan(self.L / (abs(R) + self.T/2)))
        except ZeroDivisionError:
            delta_in = self.max_deg
            delta_out = self.max_deg * 0.7

        # Aplica signo a cada servo (izq+, der- por convención)
        left = (delta_in if left_is_inner else delta_out)
        right = -(delta_out if left_is_inner else delta_in)

        # Limita por tope mecánico
        left = clamp(left, -self.max_deg, self.max_deg)
        right = clamp(right, -self.max_deg, self.max_deg)
        return left, right

    def pure_pursuit_center_angle(self, y_target):
        """
        y_target: offset lateral deseado (m). >0 a la derecha del centro del carril.
        Fórmula PP: delta = atan(2*L*y / Ld^2). Positivo = izquierda.
        Por ello usamos delta = -atan(...) para que y>0 (derecha) produzca giro a la derecha.
        """
        delta = -math.atan2(2.0 * self.L * y_target, self.Ld * self.Ld)
        return math.degrees(delta)

    def fresh(self, tstamp):
        return (time.time() - tstamp) <= self.fresh_t

    def publish_actuation(self, v, left_deg, right_deg, label):
        # Motores: mezcla diferencial simple por ángulo central equivalente
        # Si el ángulo central es grande, reduce velocidad; ya aplicamos más abajo
        lm = Float32(); rm = Float32()
        lm.data = clamp(v, -1.0, 1.0)
        rm.data = clamp(v, -1.0, 1.0)
        self.pub_lm.publish(lm); self.pub_rm.publish(rm)

        ls = Float32(); rs = Float32()
        ls.data = left_deg; rs.data = right_deg
        self.pub_ls.publish(ls); self.pub_rs.publish(rs)

        s = String(); s.data = label
        self.pub_action.publish(s)

    # ====== Decisor principal ======
    def decide(self):
        # Requiere datos frescos de visión y al menos uno de distancia
        if not (self.last_color and self.fresh(self.t_color)):
            return
        if not ((self.last_ultra and self.fresh(self.t_ultra)) or (self.last_lidar is not None and self.fresh(self.t_lidar))):
            return

        # STOP por obstáculo cercano
        obstacle = (self.last_ultra == 'obstaculo')
        if self.last_lidar is not None:
            obstacle = obstacle or (self.last_lidar <= self.stop_dist)

        if obstacle:
            self.publish_actuation(0.0, 0.0, 0.0, "STOP ⛔ (obstáculo)")
            return

        # Selección de lado según color (tu simulador usa blue para IZQ)
        # rojo -> mantenerse a la DERECHA; azul -> IZQUIERDA; pink -> parking
        color = self.last_color
        if color == 'pink':
            # Prepara maniobra de aparcamiento: baja velocidad y endereza
            self.publish_actuation(0.0, 0.0, 0.0, "Aparcar 🅿️ (preparado)")
            return

        # Offset objetivo respecto al centro del carril (m >0 = derecha)
        # Queremos que el borde exterior del coche quede separado del pilar/muro
        half_lane = self.lane_w * 0.5
        half_car  = self.W * 0.5
        y_clear = half_car + self.pillar_half + self.margin
        # Límite máximo alcanzable sin tocar: (half_lane - y_clear)
        y_mag = max(0.0, half_lane - y_clear)
        side = +1.0 if color == 'red' else (-1.0 if color == 'blue' else 0.0)
        y_target = side * y_mag  # m

        # Ángulo central con Pure Pursuit y posterior split Ackermann
        delta_center_deg = self.pure_pursuit_center_angle(y_target)
        left_deg, right_deg = self.ackermann_from_center_angle(delta_center_deg)

        # Velocidad: reduce con |delta|
        steer_factor = 1.0 - min(abs(delta_center_deg) / self.max_deg, 1.0) * 0.6
        v = clamp(self.base_speed * steer_factor, self.min_speed, self.base_speed)

        label = f"Girar {'der' if side>0 else ('izq' if side<0 else 'recto')} | δc={delta_center_deg:.1f}° | L={left_deg:.1f}° R={right_deg:.1f}° | v={v:.2f}"
        self.publish_actuation(v, left_deg, right_deg, label)

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
