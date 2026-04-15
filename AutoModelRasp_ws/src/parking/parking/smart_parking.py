#!/usr/bin/env python3
"""
Nodo Maestro de Estacionamiento Autónomo ROS 2 (Hardware Real Final).

Este nodo controla la lógica completa del vehículo Ackermann para encontrar un 
espacio y ejecutar una maniobra de estacionamiento en reversa.
Sincronizado con el driver ms200_node (Filtro EMA y array de 360 grados).
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
        
        self.pwm_crucero = 60    # Torque de patrullaje para el asfalto
        self.pwm_minimo = 45  # Deadband: Fuerza mínima para arrancar el peso del carro
        
        # --- 2. BRÚJULA DEL LIDAR (Mapeo Físico Real) ---
        self.angulo_der_rad = 3.14   # La derecha física del carro es el "Atrás" del sensor
        self.angulo_atras_rad = 4.71 # La cola física del carro es la "Derecha" del sensor
        
        # Variables de odometría global
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_actual = 0.0
        self.dist_der = 12.0
        self.dist_atras = 12.0
        
        # Memorias de referencia geométrica
        self.yaw_calle = 0.0 
        self.x_ref = 0.0
        self.y_ref = 0.0
        self.comando_dir = 1 
        
        # --- 3. ESCÁNER DE HUECOS ---
        self.p1_x = 0.0
        self.p1_y = 0.0  
        self.tamano_minimo = 0.25   # Hueco mínimo aceptado (30 cm)
        self.dist_alineacion = 0.0 # Distancia de reversa recta antes de quebrar el volante
        self.ciclos_pausa = 0
        
        # --- 4. CALIBRACIÓN DE ACTUADORES (Dirección Invertida Corregida) ---
        self.DERECHA_REVERSA = 1260  # <- Corregido: Para quebrar la llanta a la derecha
        self.CENTRO = 1500

        # --- 5. CEREBRO PI (Compensación Física Final) ---
        self.kp_dist = 50.0
        self.ki_dist = 0.0
        self.error_integral_dist = 0.0
        self.dt = 0.1

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("v34.0")

    def encoder_cb(self, msg):
        """Calcula la odometría diferencial usando los ticks reales de los motores."""
        signo = 1 if self.comando_dir == 1 else -1
        ticks_ciclo = (abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0
        dist_lineal = (ticks_ciclo / self.ticks_por_metro) * signo
        
        ds_l = (abs(msg.vel_m1) / self.ticks_por_metro) * signo
        ds_r = (abs(msg.vel_m2) / self.ticks_por_metro) * signo
        
        self.yaw_actual += (ds_r - ds_l) / self.ancho_via
        self.pos_x += dist_lineal * math.cos(self.yaw_actual)
        self.pos_y += dist_lineal * math.sin(self.yaw_actual)

    def scan_cb(self, msg):
        """
        Extrae la distancia lateral y trasera asumiendo un array de 360 puntos.
        Aplica un filtro de cono para evitar lecturas de la propia llanta.
        """
        num_rayos = len(msg.ranges)
        if num_rayos == 0: return

        idx_der = int(math.degrees(self.angulo_der_rad)) % num_rayos
        idx_atras = int(math.degrees(self.angulo_atras_rad)) % num_rayos

        def obtener_distancia_limpia(idx_central, ignora_chasis=False):
            rayos = []
            # Cono de visión de +- 10 grados para mayor robustez
            for i in range(-10, 11):
                idx = (idx_central + i) % num_rayos 
                rayos.append(msg.ranges[idx])
            
            # Filtro Espacial: Ignora lo que esté a < 0.35m a los lados (el chasis)
            min_dist = 0.35 if ignora_chasis else 0.05
            validos = [d for d in rayos if min_dist < d < 12.0 and not math.isinf(d) and not math.isnan(d)]
            
            if not validos: return 12.0
            
            # Promediamos los 3 más cercanos para eliminar puntos ciegos
            validos.sort()
            top_3 = validos[:3]
            return sum(top_3) / len(top_3)

        self.dist_der = obtener_distancia_limpia(idx_der, ignora_chasis=True)
        self.dist_atras = obtener_distancia_limpia(idx_atras, ignora_chasis=False)

    def control_loop(self):
        """Máquina de estados FSM ejecutada a 10Hz."""
        cmd = MotorCommand()
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))

        self.get_logger().info(f"[{self.estado}] Der: {self.dist_der:.2f}m | Atrás: {self.dist_atras:.2f}m", throttle_duration_sec=0.5)

        # 1. ALINEARSE A LA FILA
        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_crucero, self.CENTRO
            if self.dist_der < 1.2: 
                self.estado = 'BUSCANDO_INICIO_HUECO'

        # 2. ENCONTRAR EL BORDE DEL CARRO (Dispara cuando el láser cruza el auto)
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
            
            # Hueco lo suficientemente grande
            if dist_hueco >= self.tamano_minimo:
                self.estado = 'ALTO_TOTAL'
                self.ciclos_pausa = 0
                self.get_logger().info(f"✅ Cajón APROBADO ({dist_hueco:.2f}m). FRENANDO...")
                
            # Se acabó el hueco antes de tiempo (Carro detectado de nuevo)
            elif self.dist_der < 1.2: 
                self.estado = 'BUSCANDO_INICIO_HUECO'
                self.get_logger().info(f"❌ Hueco RECHAZADO (Muy chico: {dist_hueco:.2f}m).")

        # 4. FRENAR PARA MATAR INERCIA DEL PESO
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

        # 6. ENTRADA AL CAJÓN (Corte de volante a la derecha usando el 1260)
        elif self.estado == 'REVERSA_GIRANDO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_crucero, self.DERECHA_REVERSA
            desviacion = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))
            
            if desviacion >= 85.0 or self.dist_atras < 0.35:
                self.estado = 'CENTRADO_FINAL'
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.error_integral_dist = 0.0

        # 7. ENCLAVADO MILIMÉTRICO (Control PI + Deadband)
        elif self.estado == 'CENTRADO_FINAL':
            dist_reversa = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            error_pos = 0.60 - dist_reversa # Setpoint de profundidad en cajón
            
            self.error_integral_dist = max(-0.5, min(0.5, self.error_integral_dist + error_pos * self.dt))
            vel_control = (self.kp_dist * error_pos) + (self.ki_dist * self.error_integral_dist)
            
            vel_abs = abs(vel_control)
            
            if abs(error_pos) <= 0.015: 
                velocidad_pwm = 0
                self.estado = 'ESTACIONADO'
            else:
                # Rampa directa: Evita oscilaciones por fricción estática
                pwm_real = self.pwm_minimo + (vel_abs * 1.2) 
                velocidad_pwm = int(max(0, min(100, pwm_real)))
            
            if vel_control >= 0:
                cmd.dir_dc = 2 # Sigue en reversa
            else:
                cmd.dir_dc = 1 # Se pasó, corrige hacia adelante
                
            cmd.speed_dc = velocidad_pwm
            cmd.dir_servo = self.CENTRO 
            
            # Freno de emergencia (Lidar Trasero)
            if self.dist_atras <= 0.16:
                self.get_logger().info("Freno de emergencia trasero activado.")
                self.estado = 'ESTACIONADO'

        # 8. FIN DE LA RUTINA
        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1

        self.comando_dir = cmd.dir_dc
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