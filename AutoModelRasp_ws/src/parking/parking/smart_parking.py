import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand
from sensor_msgs.msg import LaserScan
import time
import math

class SmartParking(Node):
    def __init__(self):
        super().__init__('smart_parking')
        
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/model/carro/scan', self.scan_cb, 10)
        
        # --- NUEVO: Inicia buscando la fila de autos, no el hueco ---
        self.estado = 'BUSCANDO_FILA' 
        
        self.distancia_derecha = 0.0
        self.distancia_izq = 0.0
        self.distancia_atras = 0.0
        
        self.tiempo_inicio_estado = 0.0
        self.inicio_hueco = 0.0 
        self.tiempo_sandwich = 0.0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Piloto Automático: Navegación de 2 Fases (Búsqueda de Fila -> Búsqueda de Cajón).")

    def scan_cb(self, msg):
        num_rayos = len(msg.ranges)
        if num_rayos == 0:
            return

        def extraer_distancia(angulo_rad):
            if msg.angle_min <= angulo_rad <= msg.angle_max:
                try:
                    indice = int((angulo_rad - msg.angle_min) / msg.angle_increment)
                    rayos = msg.ranges[max(0, indice - 5) : min(num_rayos, indice + 5)]
                    
                    distancias = []
                    for d in rayos:
                        # Si lee infinito, significa espacio libre
                        if math.isinf(d) or d > 12.0 or d == 0.0:
                            distancias.append(12.0)
                        elif d > 0.05 and not math.isnan(d):
                            distancias.append(d)
                            
                    if distancias:
                        return sum(distancias) / len(distancias)
                except ZeroDivisionError:
                    pass
            return None

        d_der = extraer_distancia(3.0 * math.pi / 2.0)
        if d_der is not None:
            self.distancia_derecha = d_der

        d_izq = extraer_distancia(math.pi / 2.0)
        if d_izq is not None:
            self.distancia_izq = d_izq

        d_atras = extraer_distancia(math.pi)
        if d_atras is not None:
            self.distancia_atras = d_atras

    def control_loop(self):
        cmd = MotorCommand()
        
        # ==========================================
        # FASE 1: Buscar la fila de carros
        # ==========================================
        if self.estado == 'BUSCANDO_FILA':
            cmd.dir_dc = 1       
            cmd.speed_dc = 35    
            cmd.dir_servo = 1500 
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            # Si ve algo a menos de 80 cm a la derecha, significa que ya pasó la zona
            # libre y acaba de encontrar el primer carro estacionado (el rojo).
            if self.distancia_derecha < 0.8:
                self.get_logger().info("Fila de autos detectada. Activando escáner de cajones...")
                self.estado = 'BUSCANDO_CAJON'
                
        # ==========================================
        # FASE 2: Buscar el hueco entre los carros
        # ==========================================
        elif self.estado == 'BUSCANDO_CAJON':
            cmd.dir_dc = 1       
            cmd.speed_dc = 35    
            cmd.dir_servo = 1500 
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            # Ahora sí, busca un hueco de más de 45cm de profundidad
            if self.distancia_derecha > 0.45:
                if self.inicio_hueco == 0.0:
                    self.inicio_hueco = time.time()
                    
                # Si el hueco dura más de 0.8s, es un cajón válido
                elif time.time() - self.inicio_hueco > 0.7:
                    self.get_logger().info("¡Cajón libre detectado! Iniciando maniobra de apertura...")
                    self.estado = 'ABRIENDO_TRAYECTORIA'
                    self.tiempo_inicio_estado = time.time()
            else:
                if self.inicio_hueco != 0.0:
                    self.inicio_hueco = 0.0
                
        # ==========================================
        # FASE 3: Maniobra de Estacionamiento
        # ==========================================
        elif self.estado == 'ABRIENDO_TRAYECTORIA':
            cmd.dir_dc = 1
            cmd.speed_dc = 35
            cmd.dir_servo = 1740 
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            if time.time() - self.tiempo_inicio_estado > 3.5:
                self.estado = 'FRENANDO'
                self.tiempo_inicio_estado = time.time()

        elif self.estado == 'FRENANDO':
            cmd.dir_dc = 0
            cmd.speed_dc = 0
            cmd.dir_servo = 1500 
            cmd.turn_signals = 3 
            cmd.stop_lights = 1  
            
            if time.time() - self.tiempo_inicio_estado > 1.0:
                self.estado = 'REVERSA_CURVA'
                self.tiempo_inicio_estado = time.time()
                
        elif self.estado == 'REVERSA_CURVA':
            cmd.dir_dc = 2       
            cmd.speed_dc = 40    
            cmd.dir_servo = 1740 
            cmd.turn_signals = 3 
            cmd.stop_lights = 0
            
            if time.time() - self.tiempo_inicio_estado > 4.5:
                self.estado = 'REVERSA_RECTA'
                self.tiempo_inicio_estado = time.time()
                self.get_logger().info("Buscando autos laterales...")
                
        elif self.estado == 'REVERSA_RECTA':
            cmd.dir_dc = 2       
            cmd.speed_dc = 35
            cmd.dir_servo = 1500 
            cmd.turn_signals = 3 
            cmd.stop_lights = 0
            
            if 0.0 < self.distancia_atras <= 0.12:
                self.get_logger().info(f"¡Radar trasero activado! Freno de emergencia a {self.distancia_atras:.2f}m.")
                self.estado = 'ESTACIONADO'
                
            elif 0.0 < self.distancia_izq < 0.25 and 0.0 < self.distancia_derecha < 0.25:
                if self.tiempo_sandwich == 0.0:
                    self.tiempo_sandwich = time.time()
                    self.get_logger().info("¡Alineado con los autos! Metiendo la trompa...")
                    
                elif time.time() - self.tiempo_sandwich > 1.2:
                    self.get_logger().info("¡Perfectamente estacionado!")
                    self.estado = 'ESTACIONADO'
            else:
                if self.tiempo_sandwich != 0.0:
                    self.tiempo_sandwich = 0.0
                
        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc = 0       
            cmd.speed_dc = 0
            cmd.dir_servo = 1500 
            cmd.turn_signals = 3 
            cmd.stop_lights = 1  

        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(SmartParking())
    rclpy.shutdown()

if __name__ == '__main__':
    main()