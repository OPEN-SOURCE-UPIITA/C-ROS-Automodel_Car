#!/usr/bin/env python3
"""
Nodo Maestro de Estacionamiento (ENFOQUE HÍBRIDO).
- FASE 1 (Lazo Cerrado): Busca y mide el hueco usando LiDAR y Encoders.
- FASE 2 (Lazo Abierto): Ejecuta la maniobra de reversa basándose 100% en TIEMPOS.
- EJES: Derecha (1240), Izquierda (1740), Centro (1500). Velocidad: 65.
"""

import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
from sensor_msgs.msg import LaserScan
import math

class SmartParkingHybrid(Node):
    def __init__(self):
        super().__init__('smart_parking_hybrid')
        
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.enc_cb, 10)
        
        # --- CONFIGURACIÓN FÍSICA ---
        self.pwm_fijo = 55
        self.ticks_por_metro = 4757.0  
        self.ancho_via = 0.3210        
        
        # --- ACTUADORES ---
        self.DERECHA = 1240  
        self.IZQUIERDA = 1740 
        self.CENTRO = 1500
        
        # --- VARIABLES DE ESTADO Y SENSORES ---
        self.estado = 'BUSCANDO_CARRO'
        self.pos_x = 0.0; self.pos_y = 0.0
        self.dist_der_min = 12.0   
        self.comando_dir = 1 
        self.p1_x = 0.0; self.p1_y = 0.0
        
        # --- VARIABLES PARA EL LAZO ABIERTO (TIEMPOS) ---
        self.paso_actual = 0
        self.tiempo_inicio_paso = 0.0
        
        # Formato de secuencia: (Dirección_DC, PWM, Dirección_Servo, Segundos, Descripción)
        # dir_dc: 0=Stop, 1=Adelante, 2=Reversa
        self.secuencia = [
            (1, 65, self.CENTRO,    1.0, "1. Avanzar recto para posicionar la cola"),
            (0,  0, self.CENTRO,    0.5, "2. Freno"),
            (2, 65, self.DERECHA,   2.5, "3. Reversa girando a la derecha (Entrando)"),
            (0,  0, self.CENTRO,    0.5, "4. Freno"),
            (1, 65, self.IZQUIERDA, 0.8, "5. Adelante izquierda (Acomodo / Shimmy)"),
            (0,  0, self.CENTRO,    0.5, "6. Freno"),
            (2, 65, self.DERECHA,   1.0, "7. Reversa derecha (Completando giro)"),
            (0,  0, self.CENTRO,    0.5, "8. Freno"),
            (2, 60, self.CENTRO,    1.2, "9. Reversa recta hasta el fondo"),
            (0,  0, self.CENTRO,    999, "10. ESTACIONADO (FIN)")
        ]

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Iniciando Sistema: Detección por Sensores -> Maniobra por Tiempo.")

    def enc_cb(self, msg):
        # La odometría solo la usamos en la Fase 1 para medir el hueco
        signo = 1 if self.comando_dir == 1 else -1
        dist_lineal = (((abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0) / self.ticks_por_metro) * signo
        self.pos_x += dist_lineal # Simplificado, solo nos importa el avance lineal para medir el hueco

    def scan_cb(self, msg):
        if len(msg.ranges) != 360: return
        
        # Usamos los índices que te funcionaron bien para la detección derecha
        sector_der = msg.ranges[60:110]
        validos_der = [d for d in sector_der if d > 0.03 and not math.isnan(d)]
        self.dist_der_min = min(validos_der) if validos_der else 12.0

    def control_loop(self):
        cmd = MotorCommand()

        # ====================================================================
        # FASE 1: LAZO CERRADO (Búsqueda y medición del cajón)
        # ====================================================================
        if self.estado in ['BUSCANDO_CARRO', 'BUSCANDO_INICIO_HUECO', 'MIDIENDO_HUECO']:
            self.get_logger().info(f"[{self.estado}] Distancia Lateral: {self.dist_der_min:.2f}m", throttle_duration_sec=0.5)
            
            if self.estado == 'BUSCANDO_CARRO':
                cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
                if self.dist_der_min < 0.20: 
                    self.estado = 'BUSCANDO_INICIO_HUECO'
                    self.get_logger().info("Caja detectada. Buscando dónde termina...")

            elif self.estado == 'BUSCANDO_INICIO_HUECO':
                cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
                if self.dist_der_min > 0.40: 
                    self.p1_x = self.pos_x
                    self.estado = 'MIDIENDO_HUECO'
                    self.get_logger().info("🕳️ Hueco abierto. Midiendo longitud...")

            elif self.estado == 'MIDIENDO_HUECO':
                cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
                dist_hueco = abs(self.pos_x - self.p1_x)
                
                if dist_hueco >= 0.20: 
                    self.estado = 'INICIAR_RUTINA_TIEMPO'
                    self.get_logger().info("✅ Hueco de 20cm confirmado. Cambiando a piloto ciego (Tiempos).")
                elif self.dist_der_min < 0.15:  
                    self.estado = 'BUSCANDO_INICIO_HUECO'

        # ====================================================================
        # TRANSICIÓN DE FASE
        # ====================================================================
        elif self.estado == 'INICIAR_RUTINA_TIEMPO':
            self.paso_actual = 0
            self.tiempo_inicio_paso = self.get_clock().now().nanoseconds / 1e9
            self.estado = 'EJECUTANDO_RUTINA_TIEMPO'

        # ====================================================================
        # FASE 2: LAZO ABIERTO (Coreografía por Tiempos)
        # ====================================================================
        elif self.estado == 'EJECUTANDO_RUTINA_TIEMPO':
            ahora = self.get_clock().now().nanoseconds / 1e9
            tiempo_transcurrido = ahora - self.tiempo_inicio_paso
            
            # Extraer el comando del arreglo
            dir_dc, speed, servo, duracion, desc = self.secuencia[self.paso_actual]
            
            self.get_logger().info(f"[{self.paso_actual}] {desc} | {tiempo_transcurrido:.1f}s / {duracion}s", throttle_duration_sec=0.5)
            
            cmd.dir_dc = dir_dc
            cmd.speed_dc = speed
            cmd.dir_servo = servo
            
            # Condición de avance de paso
            if tiempo_transcurrido >= duracion:
                if self.paso_actual < len(self.secuencia) - 1:
                    self.paso_actual += 1
                    self.tiempo_inicio_paso = ahora 
                else:
                    cmd.dir_dc, cmd.speed_dc, cmd.stop_lights = 0, 0, 1

        self.comando_dir = int(cmd.dir_dc)
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init(); rclpy.spin(SmartParkingHybrid()); rclpy.shutdown()

if __name__ == '__main__':
    main()