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
        
        # --- 1. CONFIGURACIÓN FÍSICA Y MOTORES ---
        self.ticks_por_metro = 2060.0 
        self.ancho_via = 0.15 
        
        self.pwm_crucero = 85    # Fuerza para patrullar sin atascarse
        self.pwm_minimo = 60.0   # Fuerza base para romper inercia (Deadband)
        
        # --- 2. BRÚJULA DEL LIDAR (Mapeo Físico de tu Carro) ---
        # Si descubriste que "Atrás" del Lidar es la "Derecha" del carro:
        self.angulo_der_rad = 3.14   
        self.angulo_atras_rad = 1.57 
        
        # Odometría espacial
        self.pos_x = 0.0; self.pos_y = 0.0; self.yaw_actual = 0.0
        self.dist_der = 12.0; self.dist_atras = 12.0
        
        self.yaw_calle = 0.0 
        self.x_ref = 0.0; self.y_ref = 0.0; self.comando_dir = 1 
        
        # --- 3. ESCÁNER DE HUECOS ---
        self.p1_x = 0.0; self.p1_y = 0.0  
        self.tamano_minimo = 0.30   # El hueco debe medir al menos 30cm
        self.dist_alineacion = 0.35 # Cuánto retrocede recto antes de quebrar
        self.ciclos_pausa = 0
        
        # --- 4. ACTUADORES ---
        self.DERECHA_REVERSA = 1740  
        self.CENTRO = 1500

        # --- 5. CEREBRO PI (Ganancias del Auto-Tuner) ---
        self.kp_dist = 50.0
        self.ki_dist = 0.0
        self.error_integral_dist = 0.0
        self.dt = 0.1

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("v31.0 Sinergia: Optimizado para el nuevo Driver MS200 de 360°")

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

        # Gracias al nuevo driver, sabemos que num_rayos es ~360.
        # Convertimos radianes directamente a un índice de grado entero.
        idx_der = int(math.degrees(self.angulo_der_rad)) % num_rayos
        idx_atras = int(math.degrees(self.angulo_atras_rad)) % num_rayos

        def obtener_distancia_limpia(idx_central, ignora_chasis=False):
            rayos = []
            # Cono de visión de +- 10 grados (recolectamos 21 rayos)
            for i in range(-10, 11):
                idx = (idx_central + i) % num_rayos # El % hace que dé la vuelta al círculo si pasa de 360
                rayos.append(msg.ranges[idx])
            
            # Filtro Espacial: Ignora < 0.35m para no ver la propia llanta o chasis
            min_dist = 0.35 if ignora_chasis else 0.05
            validos = [d for d in rayos if min_dist < d < 12.0 and not math.isinf(d) and not math.isnan(d)]
            
            if not validos: return 12.0
            
            # Promediamos los 3 más cercanos para robustez total
            validos.sort()
            top_3 = validos[:3]
            return sum(top_3) / len(top_3)

        self.dist_der = obtener_distancia_limpia(idx_der, ignora_chasis=True)
        self.dist_atras = obtener_distancia_limpia(idx_atras, ignora_chasis=False)

    def control_loop(self):
        cmd = MotorCommand()
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))

        self.get_logger().info(f"[{self.estado}] Der: {self.dist_der:.2f}m | Atrás: {self.dist_atras:.2f}m", throttle_duration_sec=0.5)

        # 1. ALINEARSE A LA FILA
        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_crucero, self.CENTRO
            if self.dist_der < 1.2: 
                self.estado = 'BUSCANDO_INICIO_HUECO'

        # 2. ENCONTRAR EL BORDE DEL CARRO
        elif self.estado == 'BUSCANDO_INICIO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_crucero, self.CENTRO
            if self.dist_der > 1.4: 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'
                self.get_logger().info("¡Inicia hueco! Midiendo...")

        # 3. ESCANEAR CAJÓN
        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_crucero, self.CENTRO
            dist_hueco = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            
            if dist_hueco >= self.tamano_minimo:
                self.estado = 'ALTO_TOTAL'
                self.ciclos_pausa = 0
                self.get_logger().info(f"✅ Cajón APROBADO ({dist_hueco:.2f}m). FRENANDO...")
                
            elif self.dist_der < 1.2: 
                self.estado = 'BUSCANDO_INICIO_HUECO'
                self.get_logger().info(f"❌ Hueco RECHAZADO (Muy chico: {dist_hueco:.2f}m).")

        # 4. FRENAR PARA MATAR INERCIA
        elif self.estado == 'ALTO_TOTAL':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1
            self.ciclos_pausa += 1
            if self.ciclos_pausa > 10: 
                self.estado = 'REVERSA_RECTA'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y 

        # 5. RETROCEDER PARA ALINEAR LA COLA
        elif self.estado == 'REVERSA_RECTA':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_crucero, self.CENTRO
            dist_recta = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            if dist_recta >= self.dist_alineacion:
                self.yaw_calle = self.yaw_actual
                self.estado = 'REVERSA_GIRANDO'
                self.get_logger().info("Cola alineada. Quebrando volante...")

        # 6. ENTRADA AL CAJÓN
        elif self.estado == 'REVERSA_GIRANDO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_crucero, self.DERECHA_REVERSA
            desviacion = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))
            
            if desviacion >= 85.0 or self.dist_atras < 0.35:
                self.estado = 'CENTRADO_FINAL'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.error_integral_dist = 0.0

        # 7. ENCLAVADO CON PI Y RAMPA DE ENERGÍA
        elif self.estado == 'CENTRADO_FINAL':
            dist_reversa = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            error_pos = 0.60 - dist_reversa 
            
            self.error_integral_dist = max(-0.5, min(0.5, self.error_integral_dist + error_pos * self.dt))
            vel_control = (self.kp_dist * error_pos) + (self.ki_dist * self.error_integral_dist)
            
            vel_abs = abs(vel_control)
            
            if abs(error_pos) <= 0.015: 
                velocidad_pwm = 0
                self.estado = 'ESTACIONADO'
            else:
                pwm_real = self.pwm_minimo + (vel_abs * 1.2) 
                velocidad_pwm = int(max(0, min(100, pwm_real)))
            
            if vel_control >= 0:
                cmd.dir_dc = 2 
            else:
                cmd.dir_dc = 1 
                
            cmd.speed_dc = velocidad_pwm
            cmd.dir_servo = self.CENTRO 
            
            if self.dist_atras <= 0.16:
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