#!/usr/bin/env python3
"""
Nodo Maestro de Estacionamiento Autónomo ROS 2 (Hardware Real v35.0).
Incluye rutina de centrado cíclico (n veces) y protección de tipos.
"""

import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
from sensor_msgs.msg import LaserScan
import math

class SmartParking(Node):
    def __init__(self):
        super().__init__('smart_parking')
        
        # --- 0. PUBLICADORES Y SUSCRIPTORES ---
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.encoder_cb, 10)
        
        self.estado = 'BUSCANDO_CARRO' 
        
        # --- 1. CONFIGURACIÓN FÍSICA Y MOTORES ---
        self.ticks_por_metro = 2060.0 
        self.ancho_via = 0.15 
        
        self.pwm_crucero = 70    # Patrullaje lento
        self.pwm_minimo = 60   # Deadband para arrancar
        self.pwm_reacomodo = 62  # Velocidad extra baja para centrar
        
        # --- 2. BRÚJULA DEL LIDAR ---
        self.angulo_der_rad = 3.14   
        self.angulo_atras_rad = 4.71 
        
        self.pos_x = 0.0; self.pos_y = 0.0; self.yaw_actual = 0.0
        self.dist_der = 12.0; self.dist_atras = 12.0
        
        self.yaw_calle = 0.0 
        self.x_ref = 0.0; self.y_ref = 0.0; self.comando_dir = 1 
        
        # --- 3. ESCÁNER DE HUECOS ---
        self.p1_x = 0.0; self.p1_y = 0.0  
        self.tamano_minimo = 0.25   
        self.dist_alineacion = 0.0  
        self.ciclos_pausa = 0
        
        # --- 4. RUTINA DE CENTRADO (N veces) ---
        self.intentos_actuales = 0
        self.max_intentos_centrado = 2  # <--- AJUSTA AQUÍ EL NÚMERO DE REPETICIONES
        self.dist_maniobra_centrado = 0.12 # Cuánto avanza adelante antes de volver atrás

        # --- 5. CALIBRACIÓN DE ACTUADORES ---
        self.DERECHA_REVERSA = 1260  
        self.CENTRO = 1500

        # --- 6. CEREBRO PI ---
        self.kp_dist = 50.0; self.ki_dist = 0.0
        self.error_integral_dist = 0.0; self.dt = 0.1

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("v35.0 - Iniciando con Rutina de Centrado")

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
        idx_der = int(math.degrees(self.angulo_der_rad)) % num_rayos
        idx_atras = int(math.degrees(self.angulo_atras_rad)) % num_rayos

        def obtener_distancia_limpia(idx_central, ignora_chasis=False):
            rayos = []
            for i in range(-10, 11):
                idx = (idx_central + i) % num_rayos 
                rayos.append(msg.ranges[idx])
            min_dist = 0.35 if ignora_chasis else 0.05
            validos = [d for d in rayos if min_dist < d < 12.0 and not math.isinf(d) and not math.isnan(d)]
            if not validos: return 12.0
            validos.sort()
            return sum(validos[:3]) / len(validos[:3])

        self.dist_der = obtener_distancia_limpia(idx_der, ignora_chasis=True)
        self.dist_atras = obtener_distancia_limpia(idx_atras, ignora_chasis=False)

    def control_loop(self):
        cmd = MotorCommand()
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))

        self.get_logger().info(f"[{self.estado}] Intentos: {self.intentos_actuales} | Atrás: {self.dist_atras:.2f}m", throttle_duration_sec=0.5)

        # --- ESTADOS DE DETECCIÓN (IGUAL QUE ANTES) ---
        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_crucero, self.CENTRO
            if self.dist_der < 1.2: self.estado = 'BUSCANDO_INICIO_HUECO'

        elif self.estado == 'BUSCANDO_INICIO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_crucero, self.CENTRO
            if self.dist_der > 1.4: 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'

        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_crucero, self.CENTRO
            dist_hueco = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            if dist_hueco >= self.tamano_minimo:
                self.estado = 'ALTO_TOTAL'
                self.ciclos_pausa = 0
            elif self.dist_der < 1.2: self.estado = 'BUSCANDO_INICIO_HUECO'

        elif self.estado == 'ALTO_TOTAL':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            self.ciclos_pausa += 1
            if self.ciclos_pausa > 10: 
                self.estado = 'REVERSA_RECTA'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y 

        elif self.estado == 'REVERSA_RECTA':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_crucero, self.CENTRO
            if math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref) >= self.dist_alineacion:
                self.yaw_calle = self.yaw_actual
                self.estado = 'REVERSA_GIRANDO'

        # --- MODIFICACIÓN: MANIOBRA DE ENTRADA Y CENTRADO ---
        elif self.estado == 'REVERSA_GIRANDO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_crucero, self.DERECHA_REVERSA
            desviacion = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))
            
            # Si ya giró 85 grados o el Lidar detecta algo muy cerca atrás, pasamos a centrar
            if desviacion >= 85.0 or self.dist_atras < 0.20:
                self.get_logger().info("Entrada inicial completa. Iniciando rutina de centrado...")
                self.estado = 'CENTRAR_ADELANTE'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y

        elif self.estado == 'CENTRAR_ADELANTE':
            # Avanzar un poco recto para enderezar el cuerpo del carro
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_reacomodo, self.CENTRO
            dist_avanzada = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            if dist_avanzada >= self.dist_maniobra_centrado:
                self.estado = 'CENTRAR_ATRAS'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y

        elif self.estado == 'CENTRAR_ATRAS':
            # Retroceder despacio hasta el límite seguro del Lidar
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_reacomodo, self.CENTRO
            
            # Freno de seguridad o meta de profundidad
            if self.dist_atras <= 0.12:
                self.intentos_actuales += 1
                if self.intentos_actuales < self.max_intentos_centrado:
                    self.get_logger().info(f"Reacomodo {self.intentos_actuales} listo. Repitiendo...")
                    self.estado = 'CENTRAR_ADELANTE'
                    self.x_ref, self.y_ref = self.pos_x, self.pos_y
                else:
                    self.estado = 'ESTACIONADO'

        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1

        # --- PROTECCIÓN FINAL (CASTING A INT) ---
        self.comando_dir = int(cmd.dir_dc)
        cmd.dir_dc = int(cmd.dir_dc)
        cmd.speed_dc = int(cmd.speed_dc)
        cmd.dir_servo = int(cmd.dir_servo)
        
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    node = SmartParking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()