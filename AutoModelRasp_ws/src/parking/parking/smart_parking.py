#!/usr/bin/env python3
"""
Nodo Maestro de Estacionamiento Autónomo ROS 2.
- Versión 80.0: Percepción Discreta (Stop-and-Scan) y Ajustes Dinámicos.
- Evalúa si está encerrado y decide si necesita hacerse para adelante o atrás.
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
        self.pwm_fijo = 65 # Bajamos un poco para maniobras más precisas
        self.ticks_por_metro = 4757.0  
        self.ancho_via = 0.3210        
        
        # --- ACTUADORES ---
        self.DERECHA = 1260  
        self.IZQUIERDA = 1740 
        self.CENTRO = 1500

        # --- VARIABLES DE CONTROL Y ODOMETRÍA ---
        self.estado = 'BUSCANDO_CARRO'
        self.pos_x = 0.0; self.pos_y = 0.0; self.yaw_actual = 0.0
        
        # Distancias del entorno (inicializadas en valores altos)
        self.dist_frente_min = 10.0
        self.dist_atras_min = 10.0
        self.dist_izq_min = 10.0
        self.dist_der_min = 10.0 
        
        self.yaw_calle = 0.0 
        self.comando_dir = 1 
        self.x_ref = 0.0; self.y_ref = 0.0; self.p1_x = 0.0; self.p1_y = 0.0
        
        # Timer para el estado de evaluación
        self.contador_pausa = 0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🧠 v80.0 - Lógica Reactiva (Stop-and-Scan) Iniciada.")

    def enc_cb(self, msg):
        signo = 1 if self.comando_dir == 1 else -1
        dist_lineal = ((abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0 / self.ticks_por_metro) * signo
        ds_l = (abs(msg.vel_m1) / self.ticks_por_metro) * signo
        ds_r = (abs(msg.vel_m2) / self.ticks_por_metro) * signo
        self.yaw_actual += (ds_r - ds_l) / self.ancho_via
        self.pos_x += dist_lineal * math.cos(self.yaw_actual)
        self.pos_y += dist_lineal * math.sin(self.yaw_actual)

    def obtener_minimo_sector(self, ranges, centro, apertura, limite_inferior=0.05, limite_superior=10.0):
        """Función auxiliar para sacar el mínimo de un sector del LiDAR filtrando basura"""
        n = len(ranges)
        sector = []
        for i in range(centro - apertura, centro + apertura):
            idx = i % n # Permite dar la vuelta al arreglo (ej. para el frente que cruza el índice 0)
            sector.append(ranges[idx])
        validos = [d for d in sector if limite_inferior < d < limite_superior and not math.isnan(d) and not math.isinf(d)]
        return min(validos) if validos else limite_superior

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0: return
        
        # Mapeo de ángulos asumiendo 0 = Frente, n/4 = Izquierda, n/2 = Atrás, 3n/4 = Derecha
        idx_frente = 0
        idx_izq = int(n * 0.25)
        idx_atras = int(n * 0.50)
        idx_der = int(n * 0.75)
        
        # Leemos los 4 lados del coche (+/- 20 rayos de apertura para hacer un cono)
        self.dist_frente_min = self.obtener_minimo_sector(msg.ranges, idx_frente, 20)
        self.dist_atras_min  = self.obtener_minimo_sector(msg.ranges, idx_atras, 20)
        self.dist_izq_min    = self.obtener_minimo_sector(msg.ranges, idx_izq, 20)
        self.dist_der_min    = self.obtener_minimo_sector(msg.ranges, idx_der, 20)

    def control_loop(self):
        cmd = MotorCommand()
        cmd.turn_signals = 0
        cmd.stop_lights = 0
        
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))
        error_yaw = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))

        if self.estado not in ['ESTACIONADO']:
            self.get_logger().info(f"[{self.estado}] F:{self.dist_frente_min:.2f} A:{self.dist_atras_min:.2f} D:{self.dist_der_min:.2f} | Yaw: {error_yaw:.0f}°", throttle_duration_sec=0.5)

        # =================================================================
        # FASE 1: BÚSQUEDA Y MEDICIÓN DEL HUECO
        # =================================================================
        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            cmd.turn_signals = 1 # Derecha
            if self.dist_der_min < 0.95: 
                self.estado = 'BUSCANDO_INICIO_HUECO'

        elif self.estado == 'BUSCANDO_INICIO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            cmd.turn_signals = 1
            if self.dist_der_min > 1.20: 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'

        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            cmd.turn_signals = 1
            dist_recorrida = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            
            if self.dist_der_min < 0.95:
                if dist_recorrida < 0.30:
                    self.estado = 'BUSCANDO_INICIO_HUECO' # Falsa alarma
                else:
                    self.yaw_calle = self.yaw_actual
                    self.x_ref, self.y_ref = self.pos_x, self.pos_y
                    self.estado = 'AVANCE_POSICIONAMIENTO'

        elif self.estado == 'AVANCE_POSICIONAMIENTO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.CENTRO
            cmd.turn_signals = 3 # Intermitentes
            if math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref) >= 0.45:
                self.estado = 'REVERSA_GIRANDO_90'

        # =================================================================
        # FASE 2: MANIOBRA INICIAL
        # =================================================================
        elif self.estado == 'REVERSA_GIRANDO_90':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_fijo, self.DERECHA
            cmd.turn_signals = 3
            
            # Frenamos si ya cruzamos los 80 grados, o si hay algo muy cerca atrás
            if error_yaw >= 80.0 or self.dist_atras_min < 0.25:
                self.estado = 'EVALUANDO_ENTORNO'
                self.contador_pausa = 0 # Iniciamos el cronómetro de evaluación

        # =================================================================
        # FASE 3: EL CEREBRO (Percepción Discreta)
        # =================================================================
        elif self.estado == 'EVALUANDO_ENTORNO':
            # Detenemos motores y prendemos luz de freno
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1
            cmd.turn_signals = 3
            self.contador_pausa += 1
            
            # Esperamos 1 segundo (10 ciclos) para que el LiDAR mande lecturas limpias
            if self.contador_pausa > 10:
                self.get_logger().info(f"🤔 Evaluando... Atrás: {self.dist_atras_min:.2f}m, Frente: {self.dist_frente_min:.2f}m")
                
                # CONDICIÓN DE ÉXITO: Si está bien pegado atrás (<= 15cm) y no está muy chueco
                if self.dist_atras_min <= 0.15:
                    self.estado = 'ESTACIONADO'
                    
                # CONDICIÓN DE AJUSTE ADELANTE: Si estamos muy cerca de atrás pero hay espacio enfrente
                elif self.dist_atras_min < 0.18 and self.dist_frente_min > 0.20:
                    self.x_ref, self.y_ref = self.pos_x, self.pos_y
                    self.estado = 'AJUSTE_FRENTE'
                    
                # CONDICIÓN DE AJUSTE REVERSA: Si hay mucho espacio atrás (> 20cm)
                elif self.dist_atras_min >= 0.18:
                    self.x_ref, self.y_ref = self.pos_x, self.pos_y
                    self.estado = 'AJUSTE_REVERSA'

        # =================================================================
        # FASE 4: MICRO-AJUSTES
        # =================================================================
        elif self.estado == 'AJUSTE_FRENTE':
            # Avanza hacia la izquierda para enderezarse
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, self.pwm_fijo, self.IZQUIERDA
            cmd.turn_signals = 3
            dist_avanzada = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            # Se detiene si avanzó 15cm o si ya casi choca enfrente
            if dist_avanzada >= 0.15 or self.dist_frente_min < 0.15:
                self.estado = 'EVALUANDO_ENTORNO'
                self.contador_pausa = 0

        elif self.estado == 'AJUSTE_REVERSA':
            # Retrocede con volante recto para meterse más al cajón
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, self.pwm_fijo, self.CENTRO
            cmd.turn_signals = 3
            dist_retrocedida = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            # Se detiene si retrocedió 15cm o si ya casi choca atrás
            if dist_retrocedida >= 0.15 or self.dist_atras_min < 0.15:
                self.estado = 'EVALUANDO_ENTORNO'
                self.contador_pausa = 0

        # =================================================================
        # FASE FINAL
        # =================================================================
        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1
            cmd.turn_signals = 3
            self.get_logger().info("🅿️ ¡Vehículo Estacionado Correctamente!", throttle_duration_sec=2.0)

        # Preparar y publicar
        self.comando_dir = int(cmd.dir_dc)
        cmd.dir_dc = int(cmd.dir_dc)
        cmd.speed_dc = int(cmd.speed_dc)
        cmd.dir_servo = int(cmd.dir_servo)
        cmd.stop_lights = int(cmd.stop_lights)
        cmd.turn_signals = int(cmd.turn_signals)
        
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    nodo = SmartParking()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()