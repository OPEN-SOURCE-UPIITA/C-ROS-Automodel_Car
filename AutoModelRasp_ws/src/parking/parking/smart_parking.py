import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
from sensor_msgs.msg import LaserScan
import math

class SmartParking(Node):
    def __init__(self):
        super().__init__('smart_parking')
        
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.encoder_cb, 10)
        
        self.estado = 'BUSCANDO_CARRO' 
        
        # --- FÍSICA ---
        self.ticks_por_metro = 2060.0 
        self.ancho_via = 0.15 
        
        self.pos_x = 0.0; self.pos_y = 0.0; self.yaw_actual = 0.0
        self.dist_der = 12.0; self.dist_atras = 12.0
        
        self.yaw_calle = 0.0 
        self.x_ref = 0.0; self.y_ref = 0.0; self.comando_dir = 1 
        
        # --- ESCÁNER DE HUECOS ---
        self.p1_x = 0.0; self.p1_y = 0.0  
        self.tamano_minimo = 0.20  # Recuperamos tu medida ganadora
        
        # --- ACTUADORES ---
        self.DERECHA_REVERSA = 1740  
        self.CENTRO = 1500

        # --- GANANCIAS AUTO-TUNER V2 ---
        self.kp_dist = 60.0
        self.ki_dist = 5.0
        self.error_integral_dist = 0.0
        self.dt = 0.1

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"v24.0: Lidar con Filtro Anti-Chasis (Omitiendo cuerpo del robot).")

    def encoder_cb(self, msg):
        signo = 1 if self.comando_dir == 1 else -1
        ticks_ciclo = (abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0
        dist_lineal = (ticks_ciclo / self.ticks_por_metro) * signo
        
        ds_l = (abs(msg.vel_m1) / self.ticks_por_metro) * signo
        ds_r = (abs(msg.vel_m2) / self.ticks_por_metro) * signo
        
        self.yaw_actual += (ds_r - ds_l) / self.ancho_via
        self.pos_x += dist_lineal * math.cos(self.yaw_actual)
        self.pos_y += dist_lineal * math.sin(self.yaw_actual)

    def scan_cb(self, msg):
        num_rayos = len(msg.ranges)
        if num_rayos == 0: return

        idx_der = int((4.7124 - msg.angle_min) / msg.angle_increment)
        idx_atras = int((3.1416 - msg.angle_min) / msg.angle_increment)

        # ---------------------------------------------------------
        # EL TRUCO: Filtramos > 0.30m a la DERECHA para NO ver la propia llanta
        rayos_der = [d for d in msg.ranges[max(0, idx_der-20):min(num_rayos, idx_der+20)] 
                     if 0.30 < d < 12.0 and not math.isinf(d) and not math.isnan(d)]
        
        # ATRÁS lo dejamos desde 0.05m para que pueda frenar milimétricamente
        rayos_atras = [d for d in msg.ranges[max(0, idx_atras-20):min(num_rayos, idx_atras+20)] 
                       if 0.05 < d < 12.0 and not math.isinf(d) and not math.isnan(d)]
        # ---------------------------------------------------------

        self.dist_der = min(rayos_der) if rayos_der else 12.0
        self.dist_atras = min(rayos_atras) if rayos_atras else 12.0

    def control_loop(self):
        cmd = MotorCommand()
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))

        self.get_logger().info(f"[{self.estado}] Der: {self.dist_der:.2f}m", throttle_duration_sec=0.2)

        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            if self.dist_der < 1.5: 
                self.estado = 'BUSCANDO_INICIO_HUECO'

        elif self.estado == 'BUSCANDO_INICIO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            # Ahora sí, cuando el carro de a lado termine, la lectura subirá limpiamente
            if self.dist_der > 1.6: 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'
                self.get_logger().info("¡Inicia hueco! Midiendo...")

        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            dist_hueco = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            
            if dist_hueco >= self.tamano_minimo:
                self.estado = 'AVANCE_EXTRA'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y 
                self.get_logger().info(f"✅ Cajón APROBADO ({dist_hueco:.2f}m). Preparando maniobra...")
                
            elif self.dist_der < 1.4: 
                self.estado = 'BUSCANDO_INICIO_HUECO'
                self.get_logger().info(f"❌ Hueco RECHAZADO (Muy chico: {dist_hueco:.2f}m).")

        elif self.estado == 'AVANCE_EXTRA':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            dist_extra = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            if dist_extra > 0.35: 
                self.yaw_calle = self.yaw_actual
                self.estado = 'REVERSA_ENTRANDO'

        elif self.estado == 'REVERSA_ENTRANDO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, 35, self.DERECHA_REVERSA
            desviacion = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))
            
            if desviacion >= 85.0 or self.dist_atras < 0.35:
                self.estado = 'CENTRADO_FINAL'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.error_integral_dist = 0.0

        elif self.estado == 'CENTRADO_FINAL':
            dist_reversa = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            error_pos = 0.60 - dist_reversa 
            
            self.error_integral_dist = max(-0.5, min(0.5, self.error_integral_dist + error_pos * self.dt))
            vel_control = (self.kp_dist * error_pos) + (self.ki_dist * self.error_integral_dist)
            
            cmd.dir_dc = 2
            cmd.speed_dc = int(max(0, min(50, abs(vel_control))))
            cmd.dir_servo = self.CENTRO 
            
            if self.dist_atras <= 0.16 or abs(error_pos) <= 0.01:
                self.estado = 'ESTACIONADO'

        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1

        self.comando_dir = cmd.dir_dc
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init(); rclpy.spin(SmartParking()); rclpy.shutdown()

if __name__ == '__main__':
    main()