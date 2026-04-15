#!/usr/bin/env python3
"""
Nodo Maestro de Estacionamiento Autónomo ROS 2.
- Versión 54.0: DETECCIÓN ULTRA-SENSIBLE (Elimina fallos de detección).
- Ciclo de vaivén corregido y velocidad 65 PWM.
"""

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
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.enc_cb, 10)
        
        # --- CONFIGURACIÓN FÍSICA ---
        self.pwm_fijo = 70
        self.ticks_por_metro = 4757.0  
        self.ancho_via = 0.3210        
        
        # --- ACTUADORES (Ejes Originales) ---
        self.DERECHA_REVERSA = 1260  
        self.IZQUIERDA_ADELANTE = 1740 
        self.CENTRO = 1500

        # --- VARIABLES DE CONTROL ---
        self.estado = 'BUSCANDO_CARRO'
        self.pos_x = 0.0; self.pos_y = 0.0; self.yaw_actual = 0.0
        self.dist_der_max = 0.0
        self.dist_atras_min = 12.0
        self.yaw_calle = 0.0 
        self.comando_dir = 1 
        
        self.x_ref = 0.0; self.y_ref = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("📐 v54.0 - Detección Ultra-Sensible y Vaivén Activo.")

    def enc_cb(self, msg):
        signo = 1 if self.comando_dir == 1 else -1
        dist_lineal = ((abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0 / self.ticks_por_metro) * signo
        ds_l = (abs(msg.vel_m1) / self.ticks_por_metro) * signo
        ds_r = (abs(msg.vel_m2) / self.ticks_por_metro) * signo
        self.yaw_actual += (ds_r - ds_l) / self.ancho_via
        self.pos_x += dist_lineal * math.cos(self.yaw_actual)
        self.pos_y += dist_lineal * math.sin(self.yaw_actual)

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0: return
        
        # 1. ZONA DERECHA (Buscamos el hueco en un rango muy amplio)
        # Escaneamos desde el frente-derecha hasta atrás-derecha
        sector_der = msg.ranges[50:150]
        # Filtramos solo valores que no sean ruido del chasis ( > 15cm)
        validos_der = [d for d in sector_der if 0.15 < d < 10.0 and not math.isnan(d)]
        # Tomamos el valor más alto encontrado: si hay un hueco, el MAX será grande
        self.dist_der_max = max(validos_der) if validos_der else 0.0

        # 2. ZONA TRASERA (Para evitar choques en reversa)
        # Miramos los rayos del centro del arreglo (cola del carro)
        sector_atras = msg.ranges[n//2-15 : n//2+15]
        # Vemos si hay algo entre 12cm y 25cm (el otro carro)
        validos_atras = [d for d in sector_atras if 0.12 < d < 0.25]
        self.dist_atras_min = min(validos_atras) if validos_atras else 12.0

    def control_loop(self):
        cmd = MotorCommand()
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))
        error_yaw = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))

        # INFO EN TERMINAL PARA DIAGNÓSTICO
        self.get_logger().info(f"[{self.estado}] Ang: {error_yaw:.0f}° | MaxDer: {self.dist_der_max:.2f}m", throttle_duration_sec=0.5)

        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            # Si ve la primera caja (Amazon)
            if 0.15 < self.dist_der_max < 0.55: 
                self.estado = 'BUSCANDO_INICIO_HUECO'
                self.get_logger().info("📦 Primera caja detectada.")

        elif self.estado == 'BUSCANDO_INICIO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            # Si la distancia máxima salta (porque ya no hay caja)
            if self.dist_der_max > 0.65: 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'

        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            dist_recorrida = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            
            # Con que recorra 15cm de vacío es suficiente para confirmar
            if dist_recorrida >= 0.15: 
                self.yaw_calle = self.yaw_actual
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.estado = 'AVANCE_POSICIONAMIENTO'
                self.get_logger().info("✅ Hueco confirmado. Posicionando...")
            
            # Si vuelve a ver una caja antes de medir el hueco, fue un error
            elif self.dist_der_max < 0.55:
                self.estado = 'BUSCANDO_INICIO_HUECO'

        elif self.estado == 'AVANCE_POSICIONAMIENTO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            # Avanzamos 50cm para que la cola libre la caja
            if math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref) >= 0.50:
                self.estado = 'REVERSA_GIRANDO_90'
                self.get_logger().info("↩️ Iniciando Reversa...")

        elif self.estado == 'REVERSA_GIRANDO_90':
            # Reversa + Volante a la Derecha (1260)
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_fijo, self.DERECHA_REVERSA
            
            if error_yaw >= 87.0:
                self.estado = 'REVERSA_FONDO_RECTO'
            
            # SI DETECTA CAJA ATRÁS: Frenar y enderezar adelante
            elif self.dist_atras_min < 0.22:
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.estado = 'ACOMODO_ADELANTE'
                self.get_logger().warn("⚠️ Muy cerca de atrás. Maniobrando adelante...")

        elif self.estado == 'ACOMODO_ADELANTE':
            # Adelante + Volante a la Izquierda (1740)
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.IZQUIERDA_ADELANTE
            dist_avanzada = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            # Avanza 18cm para ganar espacio y reintenta
            if dist_avanzada >= 0.18 or error_yaw >= 87.0:
                self.estado = 'REVERSA_GIRANDO_90'

        elif self.estado == 'REVERSA_FONDO_RECTO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_fijo, self.CENTRO
            if self.dist_atras_min < 0.15:
                self.estado = 'ESTACIONADO'
                self.get_logger().info("🅿️ Estacionado correctamente.")

        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1

        self.comando_dir = int(cmd.dir_dc)
        cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = int(cmd.dir_dc), int(cmd.speed_dc), int(cmd.dir_servo)
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init(); rclpy.spin(SmartParking()); rclpy.shutdown()

if __name__ == '__main__':
    main()