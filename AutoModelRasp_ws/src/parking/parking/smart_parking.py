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
        self.dist_der = 10.0; self.dist_atras = 10.0
        
        self.yaw_calle = 0.0 
        self.x_ref = 0.0; self.y_ref = 0.0; self.comando_dir = 1 
        
        # --- ESCÁNER DE HUECOS ---
        self.p1_x = 0.0; self.p1_y = 0.0  
        self.tamano_minimo = 0.40  # El hueco debe medir al menos 65cm
        
        # --- ACTUADORES ---
        self.DERECHA_REVERSA = 1740  
        self.CENTRO = 1500

        # --- GANANCIAS AUTO-TUNER V2 ---
        self.kp_dist = 60.0
        self.ki_dist = 5.0
        self.error_integral_dist = 0.0
        self.dt = 0.1

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"v22.0: Escáner Dinámico (Busca huecos >= {self.tamano_minimo}m)")

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
        # Derecha (270 grados)
        idx_der = int((4.71 - msg.angle_min) / msg.angle_increment)
        rayos_der = msg.ranges[max(0, idx_der-30) : min(num_rayos, idx_der+30)]
        v_der = [d for d in rayos_der if 0.1 < d < 10.0]
        self.dist_der = sum(v_der)/len(v_der) if v_der else 10.0

        # Atrás (180 grados)
        idx_atras = int((3.14 - msg.angle_min) / msg.angle_increment)
        rayos_atras = msg.ranges[max(0, idx_atras-20) : min(num_rayos, idx_atras+20)]
        v_atras = [d for d in rayos_atras if 0.1 < d < 10.0]
        self.dist_atras = sum(v_atras)/len(v_atras) if v_atras else 10.0

    def control_loop(self):
        cmd = MotorCommand()
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))

        self.get_logger().info(f"[{self.estado}] Der: {self.dist_der:.2f}m", throttle_duration_sec=0.5)

        # 1. ASEGURAR ALINEACIÓN A LA FILA
        # Umbral relajado a 1.2m para no ser tan estrictos
        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            if self.dist_der < 1.2: 
                self.estado = 'BUSCANDO_INICIO_HUECO'

        # 2. ESPERAR A QUE TERMINE EL CARRO ACTUAL (Marca el Punto 1)
        # Umbral subido a 1.5m para asegurar que de verdad se acabó el carro
        elif self.estado == 'BUSCANDO_INICIO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            if self.dist_der > 1.5: 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'
                self.get_logger().info("Hueco detectado. Midiendo...")

        # 3. ESCANEAR EL HUECO EN TIEMPO REAL
        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            dist_hueco = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            
            # CASO A: El hueco ya cumplió el tamaño mínimo. ¡Lo tomamos!
            if dist_hueco >= self.tamano_minimo:
                self.estado = 'AVANCE_EXTRA'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y # Guardamos posición actual para el avance
                self.get_logger().info(f"Cajón APROBADO ({dist_hueco:.2f}m). Preparando maniobra...")
                
            # CASO B: Vuelve a ver lámina (< 1.2m) ANTES de llegar al tamaño mínimo
            elif self.dist_der < 1.2: 
                self.estado = 'BUSCANDO_INICIO_HUECO'
                self.get_logger().info(f"Hueco RECHAZADO (Muy chico: {dist_hueco:.2f}m).")

        # 4. AVANCE EXTRA DE HOLGURA (Sin Swing)
        # Como no tenemos Swing, avanzamos 25cm extra para librar bien la trompa antes de meter reversa
        elif self.estado == 'AVANCE_EXTRA':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, 30, self.CENTRO
            dist_extra = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            if dist_extra > 0.25: 
                self.yaw_calle = self.yaw_actual
                self.estado = 'REVERSA_ENTRANDO'

        # 5. REVERSA DIRECTA AL CAJÓN
        elif self.estado == 'REVERSA_ENTRANDO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, 35, self.DERECHA_REVERSA
            desviacion = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))
            
            if desviacion >= 85.0 or self.dist_atras < 0.35:
                self.estado = 'CENTRADO_FINAL'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.error_integral_dist = 0.0

        # 6. ENCLAVADO CON PI
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