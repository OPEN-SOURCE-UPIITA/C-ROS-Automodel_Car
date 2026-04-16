#!/usr/bin/env python3
"""
Nodo Maestro de Estacionamiento Autónomo ROS 2.
- Versión 100.0: Control de Velocidad en Lazo Cerrado (Encoder-Adaptive).
"""

import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math
import time # Necesario para calcular la velocidad real

class SmartParking(Node):
    def __init__(self):
        super().__init__('smart_parking')
        
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data) 
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.enc_cb, 10)
        
        # --- CONFIGURACIÓN FÍSICA Y CINEMÁTICA ---
        self.ticks_por_metro = 4757.0  
        self.ancho_via = 0.3210        
        
        # --- CONTROL DE VELOCIDAD ADAPTATIVA (PID PROPORCIONAL) ---
        self.vel_objetivo = 0.20       # Queremos ir siempre a 20 cm por segundo (0.20 m/s)
        self.vel_actual = 0.0          # Velocidad leída por los encoders
        self.pwm_base = 70.0           # PWM mínimo estimado para que el auto se mueva
        self.Kp = 120.0                # Qué tan agresivo compensa si va lento o rápido
        self.last_time = time.time()   # Reloj para el Delta de Tiempo
        
        # --- ACTUADORES ---
        self.DERECHA_REVERSA = 1260  
        self.IZQUIERDA_ADELANTE = 1740 
        self.CENTRO = 1500

        # --- VARIABLES DE CONTROL Y ODOMETRÍA ---
        self.estado = 'BUSCANDO_CARRO'
        self.pos_x = 0.0; self.pos_y = 0.0; self.yaw_actual = 0.0
        
        self.dist_frente_min = 10.0; self.dist_atras_min = 10.0
        self.dist_izq_min = 10.0; self.dist_der_min = 10.0 
        self.dist_carro_ref = 10.0 
        
        self.yaw_calle = 0.0 
        self.comando_dir = 1 
        self.x_ref = 0.0; self.y_ref = 0.0; self.p1_x = 0.0; self.p1_y = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🏎️ v100.0 - Velocidad Adaptativa Lazo Cerrado Activada.")

    def enc_cb(self, msg):
        # 1. Calculamos Odometría (Posición)
        signo = 1 if self.comando_dir == 1 else -1
        dist_lineal = ((abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0 / self.ticks_por_metro)
        ds_l = (abs(msg.vel_m1) / self.ticks_por_metro) * signo
        ds_r = (abs(msg.vel_m2) / self.ticks_por_metro) * signo
        self.yaw_actual += (ds_r - ds_l) / self.ancho_via
        self.pos_x += (dist_lineal * signo) * math.cos(self.yaw_actual)
        self.pos_y += (dist_lineal * signo) * math.sin(self.yaw_actual)

        # 2. Calculamos Velocidad Real (m/s)
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            self.vel_actual = dist_lineal / dt
        self.last_time = current_time

    def obtener_pwm_adaptativo(self):
        """Calcula el PWM exacto para mantener los 0.20 m/s sin importar la batería"""
        error_vel = self.vel_objetivo - self.vel_actual
        pwm_calculado = self.pwm_base + (error_vel * self.Kp)
        
        # Limitamos para que nunca se pase del 100% ni baje del límite de fricción (ej. 40%)
        pwm_seguro = max(40.0, min(95.0, pwm_calculado))
        return int(pwm_seguro)

    def obtener_minimo_sector(self, ranges, centro, apertura, limite_inferior=0.05, limite_superior=10.0):
        n = len(ranges)
        sector = []
        for i in range(centro - apertura, centro + apertura):
            idx = i % n 
            sector.append(ranges[idx])
        validos = [d for d in sector if limite_inferior < d < limite_superior and not math.isnan(d) and not math.isinf(d)]
        return min(validos) if validos else limite_superior

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0: return
        idx_frente = 0; idx_izq = int(n * 0.25); idx_atras = int(n * 0.50); idx_der = int(n * 0.75)
        self.dist_frente_min = self.obtener_minimo_sector(msg.ranges, idx_frente, 20)
        self.dist_atras_min  = self.obtener_minimo_sector(msg.ranges, idx_atras, 20)
        self.dist_izq_min    = self.obtener_minimo_sector(msg.ranges, idx_izq, 20)
        self.dist_der_min    = self.obtener_minimo_sector(msg.ranges, idx_der, 20)

    def control_loop(self):
        cmd = MotorCommand()
        cmd.turn_signals = 0; cmd.stop_lights = 0
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))
        error_yaw = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))
        
        # Obtenemos el PWM ajustado de este instante
        pwm_dinamico = self.obtener_pwm_adaptativo()

        if self.estado not in ['ESTACIONADO']:
            self.get_logger().info(f"[{self.estado}] V:{self.vel_actual:.2f}m/s -> PWM:{pwm_dinamico} | D:{self.dist_der_min:.2f}m", throttle_duration_sec=0.5)

        # =================================================================
        # FASE 1: BÚSQUEDA DEL ANCHO DEL CAJÓN
        # =================================================================
        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 1 
            if self.dist_der_min < 1.50: 
                self.dist_carro_ref = self.dist_der_min 
                self.estado = 'SIGUIENDO_CARRO'

        elif self.estado == 'SIGUIENDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 1
            if self.dist_der_min < self.dist_carro_ref:
                self.dist_carro_ref = self.dist_der_min
                
            if self.dist_der_min > (self.dist_carro_ref + 0.30): 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'

        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 1
            dist_recorrida = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            
            if self.dist_der_min < (self.dist_carro_ref + 0.20) or dist_recorrida >= 0.35:
                if dist_recorrida < 0.25:
                    self.estado = 'BUSCANDO_CARRO' 
                else:
                    self.yaw_calle = self.yaw_actual
                    self.x_ref, self.y_ref = self.pos_x, self.pos_y
                    self.estado = 'AVANCE_POSICIONAMIENTO'

        elif self.estado == 'AVANCE_POSICIONAMIENTO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 3 
            if math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref) >= 0.25:
                self.estado = 'REVERSA_GIRANDO_90'

        # =================================================================
        # FASE 2: MANIOBRA
        # =================================================================
        elif self.estado == 'REVERSA_GIRANDO_90':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, pwm_dinamico, self.DERECHA_REVERSA
            cmd.turn_signals = 3
            
            if error_yaw >= 85.0:
                self.estado = 'REVERSA_FONDO_RECTO'
            elif self.dist_atras_min < 0.35 or self.dist_der_min < 0.12:
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.estado = 'ACOMODO_ADELANTE'

        elif self.estado == 'ACOMODO_ADELANTE':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.IZQUIERDA_ADELANTE
            cmd.turn_signals = 3
            dist_avanzada = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            if dist_avanzada >= 0.20 or error_yaw >= 85.0 or self.dist_frente_min < 0.15:
                self.estado = 'REVERSA_GIRANDO_90'
                
        elif self.estado == 'REVERSA_FONDO_RECTO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, pwm_dinamico, self.CENTRO
            if self.dist_atras_min < 0.20:
                self.estado = 'ESTACIONADO'

        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1; cmd.turn_signals = 3

        # Preparar y publicar
        self.comando_dir = int(cmd.dir_dc)
        cmd.dir_dc = int(cmd.dir_dc); cmd.speed_dc = int(cmd.speed_dc); cmd.dir_servo = int(cmd.dir_servo)
        cmd.stop_lights = int(cmd.stop_lights); cmd.turn_signals = int(cmd.turn_signals)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args); nodo = SmartParking()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally: nodo.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()