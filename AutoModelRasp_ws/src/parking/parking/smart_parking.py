import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand
from sensor_msgs.msg import LaserScan
import time
import math

class SmartParking(Node):
    def __init__(self):
        super().__init__('smart_parking')
        
        # Publicador para mandar los comandos al carro
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        
        # Suscriptor al LiDAR. 
        # NOTA: En la simulación usa '/model/carro/scan'. 
        # Para el carro físico con el MS200, cámbialo a '/scan'
        self.scan_sub = self.create_subscription(LaserScan, '/model/carro/scan', self.scan_cb, 10)
        
        # Variables de estado
        self.estado = 'BUSCANDO'
        self.distancia_derecha = 0.0
        self.tiempo_inicio_estado = 0.0
        
        # Bucle de control (se ejecuta 10 veces por segundo)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Piloto Automático Iniciado: Buscando lugar en batería...")

    def scan_cb(self, msg):
        num_rayos = len(msg.ranges)
        
        if num_rayos > 0:
            # Buscamos la derecha del robot. En un escaneo de 0 a 360 grados, 
            # la derecha se encuentra típicamente a los 270 grados (3 * PI / 2).
            # Si en la vida real atornillas el LiDAR volteado, ajusta este valor.
            angulo_objetivo = 3.0 * math.pi / 2.0
            
            # Verificamos que el ángulo esté dentro del rango de lectura
            if msg.angle_min <= angulo_objetivo <= msg.angle_max:
                try:
                    # Fórmula para saber qué rayo exacto apunta a la derecha
                    indice_central = int((angulo_objetivo - msg.angle_min) / msg.angle_increment)
                    
                    # Tomamos un "cono" de 5 rayos antes y 5 después para mayor estabilidad
                    inicio = max(0, indice_central - 5)
                    fin = min(num_rayos, indice_central + 5)
                    
                    rayos_derecha = msg.ranges[inicio:fin]
                    
                    # Filtramos basura: Distancias menores a 10cm, mayores a 12m (rango MS200), infinitos o NaN
                    distancias_validas = [
                        d for d in rayos_derecha 
                        if 0.1 < d < 12.0 and not math.isinf(d) and not math.isnan(d)
                    ]
                    
                    if distancias_validas:
                        # Sacamos el promedio de la distancia
                        self.distancia_derecha = sum(distancias_validas) / len(distancias_validas)
                        
                except ZeroDivisionError:
                    pass # Evita un crasheo si Gazebo arranca enviando angle_increment = 0

    def control_loop(self):
        cmd = MotorCommand()
        
        if self.estado == 'BUSCANDO':
            cmd.dir_dc = 1       
            cmd.speed_dc = 35    
            cmd.dir_servo = 1500 
            cmd.turn_signals = 1 # Direccional derecha (avisa que busca lugar)
            cmd.stop_lights = 0
            
            # El fondo del cajón permite hasta 60cm. Si lee > 0.45m, sabemos que hay un hueco.
            if self.distancia_derecha > 0.45:
                self.get_logger().info(f"¡Cajón libre detectado! Profundidad: {self.distancia_derecha:.2f}m. Preparando maniobra...")
                self.estado = 'ALINEANDO'
                self.tiempo_inicio_estado = time.time()
                
        elif self.estado == 'ALINEANDO':
            cmd.dir_dc = 1
            cmd.speed_dc = 35
            cmd.dir_servo = 1500
            cmd.turn_signals = 1 # Mantiene direccional derecha
            cmd.stop_lights = 0
            
            # El cajón a escala mide solo 30cm de ancho. Avanzamos muy poco (0.6 seg) 
            # para que el eje trasero rebase el cajón y quede en posición.
            if time.time() - self.tiempo_inicio_estado > 0.6:
                self.estado = 'FRENANDO'
                self.tiempo_inicio_estado = time.time()

        elif self.estado == 'FRENANDO':
            cmd.dir_dc = 0
            cmd.speed_dc = 0
            cmd.dir_servo = 1500
            cmd.turn_signals = 3 # Enciende Intermitentes de emergencia
            cmd.stop_lights = 1  # Enciende luces de Stop (freno rojo)
            
            # Pausa de 1 segundo para asegurar que el coche se detuvo físicamente por inercia
            if time.time() - self.tiempo_inicio_estado > 1.0:
                self.estado = 'REVERSA_CURVA'
                self.tiempo_inicio_estado = time.time()
                
        elif self.estado == 'REVERSA_CURVA':
            cmd.dir_dc = 2       # Reversa
            cmd.speed_dc = 40    # Un poco más de fuerza para mover el auto detenido
            cmd.dir_servo = 1740 # Giro de llantas a la derecha a tope para meter la cola
            cmd.turn_signals = 3 # Mantiene intermitentes
            cmd.stop_lights = 0
            
            # Retrocede girando hasta quedar perpendicular a la banqueta (Aprox 90 grados)
            # *Este tiempo es el más importante a calibrar en pruebas*
            if time.time() - self.tiempo_inicio_estado > 1.5:
                self.estado = 'REVERSA_RECTA'
                self.tiempo_inicio_estado = time.time()
                
        elif self.estado == 'REVERSA_RECTA':
            cmd.dir_dc = 2       # Sigue en reversa
            cmd.speed_dc = 35
            cmd.dir_servo = 1500 # Endereza las llantas
            cmd.turn_signals = 3 # Mantiene intermitentes
            cmd.stop_lights = 0
            
            # Se mete recto hasta el fondo del cajón (Aprovechando la profundidad)
            if time.time() - self.tiempo_inicio_estado > 1.0:
                self.estado = 'ESTACIONADO'
                
        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc = 0       # Stop motores
            cmd.speed_dc = 0
            cmd.dir_servo = 1500 # Llantas al centro
            cmd.turn_signals = 3 # Deja las intermitentes prendidas indicando maniobra terminada
            cmd.stop_lights = 1  # Freno puesto
            self.get_logger().info("¡Aston Martin estacionado a escala correctamente!")

        # Enviamos el mensaje publicado al sistema ROS 2
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(SmartParking())
    rclpy.shutdown()

if __name__ == '__main__':
    main()