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
        
        self.estado = 'BUSCANDO_FILA' 
        
        # Constantes mecánicas
        self.ticks_por_metro = 2060.0 
        self.distancia_eje = 0.15 
        
        # Odometría
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_actual = 0.0
        
        # Sensores
        self.distancia_derecha = 12.0
        self.distancia_izquierda = 12.0 
        self.distancia_atras = 12.0
        
        # Referencias
        self.yaw_calle = 0.0 
        self.x_ref = None
        self.y_ref = None 
        self.comando_dir = 1 
        
        # Mapeo de Servo (Ajustado según tus observaciones)
        self.IZQUIERDA = 1740
        self.DERECHA = 1260
        self.CENTRO = 1500

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Piloto v10.1: Frenado Seguro y Doble Condición Activados.")

    def encoder_cb(self, msg):
        signo = 1 if self.comando_dir == 1 else -1
        ds_l = (abs(msg.vel_m1) / self.ticks_por_metro) * signo
        ds_r = (abs(msg.vel_m2) / self.ticks_por_metro) * signo
        distancia_lineal = (ds_l + ds_r) / 2.0
        
        if abs(distancia_lineal) > 0.10: return
            
        delta_yaw = (ds_r - ds_l) / self.distancia_eje
        self.yaw_actual += delta_yaw
        self.pos_x += distancia_lineal * math.cos(self.yaw_actual)
        self.pos_y += distancia_lineal * math.sin(self.yaw_actual)

    def scan_cb(self, msg):
        num_rayos = len(msg.ranges)
        if num_rayos == 0: return

        def extraer_distancia(angulo_rad):
            diferencia = math.atan2(math.sin(angulo_rad - msg.angle_min), math.cos(angulo_rad - msg.angle_min))
            if diferencia < 0: diferencia += 2.0 * math.pi
            try:
                indice = int(diferencia / msg.angle_increment)
                if indice >= num_rayos: indice = num_rayos - 1
                rayos = msg.ranges[max(0, indice - 5) : min(num_rayos, indice + 5)]
                distancias = [d for d in rayos if not (math.isinf(d) or math.isnan(d) or d > 12.0 or d <= 0.20)]
                if distancias: return sum(distancias) / len(distancias)
            except ZeroDivisionError: pass
            return 12.0

        self.distancia_derecha = extraer_distancia(3.0 * math.pi / 2.0)  
        self.distancia_izquierda = extraer_distancia(math.pi / 2.0)      
        self.distancia_atras = extraer_distancia(math.pi)                

    def control_loop(self):
        cmd = MotorCommand()
        def diferencia_angular(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))

        # Telemetría
        self.get_logger().info(f"[{self.estado}] Yaw: {math.degrees(self.yaw_actual):.1f}° | Der: {self.distancia_derecha:.2f}m", throttle_duration_sec=0.5)

        if self.estado == 'BUSCANDO_FILA':
            cmd.dir_dc = 1; cmd.speed_dc = 35; cmd.dir_servo = self.CENTRO; cmd.turn_signals = 1
            if self.distancia_derecha < 0.8:
                self.estado = 'BUSCANDO_CAJON'
                
        elif self.estado == 'BUSCANDO_CAJON':
            cmd.dir_dc = 1; cmd.speed_dc = 35; cmd.dir_servo = self.CENTRO
            if self.distancia_derecha > 0.50: 
                if self.x_ref is None:
                    self.x_ref = self.pos_x; self.y_ref = self.pos_y
                distancia = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
                if distancia > 0.70: 
                    self.estado = 'FRENADO_DETECCION'
                    self.x_ref = self.pos_x; self.y_ref = self.pos_y
            else:
                self.x_ref = None 

        elif self.estado == 'FRENADO_DETECCION':
            cmd.dir_dc = 0; cmd.speed_dc = 0; cmd.dir_servo = self.CENTRO 
            self.estado = 'ACOMODO_INICIAL'
            self.x_ref = self.pos_x; self.y_ref = self.pos_y
        
        elif self.estado  == 'ACOMODO_INICIAL':
            cmd.dir_dc = 2; cmd.speed_dc = 30; cmd.dir_servo = self.CENTRO
            distancia = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            if distancia > 0.90:
                self.estado = 'FRENADO_INICIAL'
                self.x_ref = self.pos_x; self.y_ref = self.pos_y
        
        elif self.estado == 'FRENADO_INICIAL':
            cmd.dir_dc = 0; cmd.speed_dc = 0; cmd.dir_servo = self.IZQUIERDA
            self.yaw_calle = self.yaw_actual
            self.estado = 'ABRIENDO_SWING'
        
        elif self.estado == 'ABRIENDO_SWING':
            cmd.dir_dc = 1; cmd.speed_dc = 40; cmd.dir_servo = self.IZQUIERDA
            distancia = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            if distancia > 1.95:
                self.estado = 'FRENADO_PRE_REVERSA'
                self.x_ref = self.pos_x; self.y_ref = self.pos_y

        elif self.estado == 'FRENADO_PRE_REVERSA':
            cmd.dir_dc = 0; cmd.speed_dc = 0; cmd.dir_servo = self.IZQUIERDA
            self.yaw_calle = self.yaw_actual
            self.estado = 'REVERSA_ENTRANDO'

        elif self.estado == 'REVERSA_ENTRANDO':
            cmd.dir_dc = 2; cmd.speed_dc = 30; cmd.dir_servo = self.IZQUIERDA
            desviacion = abs(math.degrees(diferencia_angular(self.yaw_actual, self.yaw_calle)))
            
            if desviacion >= 105.0 or self.distancia_atras <= 0.25:
                self.estado = 'FRENADO_FINAL'
                # Guardamos la X y Y exactas de cuando empieza a retroceder recto
                self.x_ref = self.pos_x; self.y_ref = self.pos_y

        elif self.estado == 'FRENADO_FINAL':
            cmd.dir_dc = 0; cmd.speed_dc = 0; cmd.dir_servo = self.IZQUIERDA
            self.yaw_calle = self.yaw_actual
            self.estado = 'REVERSA'

        elif self.estado == 'REVERSA':
            if self.distancia_izquierda < 0.50 and self.distancia_derecha < 0.50:
                error = self.distancia_izquierda - self.distancia_derecha
                Kp = 4000.0 
                correccion = int(Kp * error)
                correccion = max(-200, min(200, correccion))
                pwm_inteligente = self.CENTRO + correccion
            else:
                pwm_inteligente = self.CENTRO

            cmd.dir_dc = 2; cmd.speed_dc = 30; cmd.dir_servo = pwm_inteligente
            
            # Calculamos la distancia que ha recorrido en reversa recta
            dist_reversa = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            # --- DOBLE SEGURO DE FRENADO ---
            # Frena si el láser ve la pared a 25cm O si ya recorrió 65cm hacia adentro del cajón
            if self.distancia_atras <= 0.25 or dist_reversa >= 0.65:
                self.estado = 'REACOMODO'
        
        elif self.estado == 'REACOMODO':
            cmd.dir_dc = 1; cmd.speed_dc = 40; cmd.dir_servo = self.IZQUIERDA
            distancia = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            if distancia > 2.5:
                self.estado = 'REVERSA_FINAL'
                self.x_ref = self.pos_x; self.y_ref = self.pos_y
                
        elif self.estado == 'REVERSA_FINAL':
            if self.distancia_izquierda < 0.50 and self.distancia_derecha < 0.50:
                error = self.distancia_izquierda - self.distancia_derecha
                Kp = 4000.0 
                correccion = int(Kp * error)
                correccion = max(-200, min(200, correccion))
                pwm_inteligente = self.CENTRO + correccion
            else:
                pwm_inteligente = self.CENTRO

            cmd.dir_dc = 2; cmd.speed_dc = 30; cmd.dir_servo = pwm_inteligente
            
            # Calculamos la distancia que ha recorrido en reversa recta
            dist_reversa = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            # --- DOBLE SEGURO DE FRENADO ---
            # Frena si el láser ve la pared a 25cm O si ya recorrió 65cm hacia adentro del cajón
            if self.distancia_atras <= 0.25 or dist_reversa >= 0.65:
                self.estado = 'ESTACIONADO'
                
        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc = 0; cmd.speed_dc = 0; cmd.dir_servo = self.CENTRO; cmd.stop_lights = 1  

        self.comando_dir = cmd.dir_dc
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init(); rclpy.spin(SmartParking()); rclpy.shutdown()

if __name__ == '__main__':
    main()