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
        
        self.estado = 'BUSCANDO'
        self.distancia_derecha = 0.0
        self.tiempo_inicio_estado = 0.0
        self.inicio_hueco = 0.0 
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Piloto Automático: Buscando lugar con maniobra de apertura...")

    def scan_cb(self, msg):
        num_rayos = len(msg.ranges)
        
        if num_rayos > 0:
            angulo_objetivo = 3.0 * math.pi / 2.0
            
            if msg.angle_min <= angulo_objetivo <= msg.angle_max:
                try:
                    indice_central = int((angulo_objetivo - msg.angle_min) / msg.angle_increment)
                    inicio = max(0, indice_central - 5)
                    fin = min(num_rayos, indice_central + 5)
                    
                    rayos_derecha = msg.ranges[inicio:fin]
                    
                    distancias_validas = [
                        d for d in rayos_derecha 
                        if 0.1 < d < 12.0 and not math.isinf(d) and not math.isnan(d)
                    ]
                    
                    if distancias_validas:
                        self.distancia_derecha = sum(distancias_validas) / len(distancias_validas)
                        
                except ZeroDivisionError:
                    pass

    def control_loop(self):
        cmd = MotorCommand()
        
        if self.estado == 'BUSCANDO':
            cmd.dir_dc = 1       
            cmd.speed_dc = 35    
            cmd.dir_servo = 1500 
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            if self.distancia_derecha > 0.45:
                if self.inicio_hueco == 0.0:
                    self.inicio_hueco = time.time()
                    
                elif time.time() - self.inicio_hueco > 0.8:
                    self.get_logger().info("¡Cajón detectado! Abriendo trayectoria a la izquierda...")
                    self.estado = 'ABRIENDO_TRAYECTORIA'
                    self.tiempo_inicio_estado = time.time()
            else:
                if self.inicio_hueco != 0.0:
                    self.inicio_hueco = 0.0
                
        elif self.estado == 'ABRIENDO_TRAYECTORIA':
            # 1. Avanza y gira a la izquierda para acomodar la cola
            cmd.dir_dc = 1
            cmd.speed_dc = 35
            cmd.dir_servo = 1740 # <-- GIRO A LA IZQUIERDA
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            # CALIBRACIÓN: Qué tanto avanza hacia el frente/izquierda
            if time.time() - self.tiempo_inicio_estado > 3.5:
                self.estado = 'FRENANDO'
                self.tiempo_inicio_estado = time.time()

        elif self.estado == 'FRENANDO':
            # 2. Se detiene por completo antes del cambio de velocidad
            cmd.dir_dc = 0
            cmd.speed_dc = 0
            cmd.dir_servo = 1500 # Endereza momentáneamente
            cmd.turn_signals = 3 
            cmd.stop_lights = 1  
            
            if time.time() - self.tiempo_inicio_estado > 1.0:
                self.estado = 'REVERSA_CURVA'
                self.tiempo_inicio_estado = time.time()
                
        elif self.estado == 'FRENANDO':
            # 2. Se detiene por completo antes del cambio de velocidad
            cmd.dir_dc = 2
            cmd.speed_dc = 40
            cmd.dir_servo = 1500 # Endereza momentáneamente
            cmd.turn_signals = 3 
            cmd.stop_lights = 1  
            
            if time.time() - self.tiempo_inicio_estado > 1.5:
                self.estado = 'REVERSA_RECTA'
                self.tiempo_inicio_estado = time.time()
                
        elif self.estado == 'REVERSA_CURVA':
            # 3. Retrocede metiendo la cola al cajón
            cmd.dir_dc = 2       
            cmd.speed_dc = 40    
            cmd.dir_servo = 1740 # <-- GIRO A LA DERECHA (hacia el cajón)
            cmd.turn_signals = 3 
            cmd.stop_lights = 0
            
            # CALIBRACIÓN: Cuánto tiempo gira en reversa hasta quedar perpendicular a la calle
            if time.time() - self.tiempo_inicio_estado > 4.5:
                self.estado = 'REVERSA_RECTA'
                self.tiempo_inicio_estado = time.time()
                
        elif self.estado == 'REVERSA_RECTA':
            # 4. Endereza y empuja hasta el fondo
            cmd.dir_dc = 2       
            cmd.speed_dc = 35
            cmd.dir_servo = 1500 # <-- ENDEREZA LLANTAS
            cmd.turn_signals = 3 
            cmd.stop_lights = 0
            
            # CALIBRACIÓN: Cuánto tiempo retrocede recto
            if time.time() - self.tiempo_inicio_estado > 2.5:
                self.estado = 'ESTACIONADO'
                
        elif self.estado == 'ESTACIONADO':
            # 5. Maniobra terminada
            cmd.dir_dc = 0       
            cmd.speed_dc = 0
            cmd.dir_servo = 1500 
            cmd.turn_signals = 3 
            cmd.stop_lights = 1  
            self.get_logger().info("¡Maniobra completada con éxito!")

        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(SmartParking())
    rclpy.shutdown()

if __name__ == '__main__':
    main()