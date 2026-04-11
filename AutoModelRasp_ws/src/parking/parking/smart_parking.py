import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
import math

class SmartParking(Node):
    def __init__(self):
        super().__init__('smart_parking')
        
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/model/carro/scan', self.scan_cb, 10)
        
        # --- NUEVO: Suscripción a Odometría ---
        self.odom_sub = self.create_subscription(Odometry, '/model/carro/odometry', self.odom_cb, 10)
        
        self.estado = 'BUSCANDO_FILA' 
        
        # --- Variables de Sensores ---
        self.distancia_derecha = 0.0
        self.distancia_izq = 0.0
        self.distancia_atras = 0.0
        
        # --- Variables Inerciales (IMU/Odometría) ---
        self.yaw_actual = 0.0
        self.yaw_calle = 0.0 # Guardará la orientación inicial de la calle recta
        
        # --- Cronómetros ---
        self.tiempo_inicio_estado = 0.0
        self.inicio_hueco = 0.0 
        self.tiempo_sandwich = 0.0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Piloto Automático v3: Navegación Relativa + Odometría de Precisión activada.")

    # =======================================================
    # 1. LECTURA DE ODOMETRÍA (El "Oído Interno" del carro)
    # =======================================================
    def odom_cb(self, msg):
        # Extraemos la orientación en formato Cuaternión
        q = msg.pose.pose.orientation
        
        # Función mágica para convertir Cuaternión a Radianes (Ángulo Yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw_actual = math.atan2(siny_cosp, cosy_cosp)
        
    # Función auxiliar para calcular la diferencia de ángulos sin volvernos locos con los signos
    def diferencia_angular(self, angulo1, angulo2):
        diff = angulo1 - angulo2
        return math.atan2(math.sin(diff), math.cos(diff))

    # =======================================================
    # 2. LECTURA DE LiDAR (Los "Ojos" del carro)
    # =======================================================
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

    # =======================================================
    # 3. EL CEREBRO: MÁQUINA DE ESTADOS
    # =======================================================
    def control_loop(self):
        cmd = MotorCommand()
        
        if self.estado == 'BUSCANDO_FILA':
            cmd.dir_dc = 1       
            cmd.speed_dc = 35    
            cmd.dir_servo = 1500 
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            if self.distancia_derecha < 0.8:
                self.get_logger().info("Fila de autos detectada. Escaneando cajones...")
                self.estado = 'BUSCANDO_CAJON'
                
        elif self.estado == 'BUSCANDO_CAJON':
            cmd.dir_dc = 1       
            cmd.speed_dc = 35    
            cmd.dir_servo = 1500 
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            if self.distancia_derecha > 0.45:
                if self.inicio_hueco == 0.0:
                    self.inicio_hueco = time.time()
                    
                elif time.time() - self.inicio_hueco > 0.8:
                    self.get_logger().info("¡Cajón libre! Calculando ángulos de apertura...")
                    
                    # ¡AQUÍ GUARDAMOS LA BRÚJULA! (Orientación de la calle recta)
                    self.yaw_calle = self.yaw_actual 
                    
                    self.estado = 'ABRIENDO_TRAYECTORIA'
                    self.tiempo_inicio_estado = time.time()
            else:
                if self.inicio_hueco != 0.0:
                    self.inicio_hueco = 0.0
                
        elif self.estado == 'ABRIENDO_TRAYECTORIA':
            cmd.dir_dc = 1
            cmd.speed_dc = 35
            cmd.dir_servo = 1740 # Izquierda
            cmd.turn_signals = 1 
            cmd.stop_lights = 0
            
            # --- CIERRE DE LAZO: ODOMETRÍA ---
            # ¿Cuántos grados nos hemos desviado de la calle?
            desviacion_rad = abs(self.diferencia_angular(self.yaw_actual, self.yaw_calle))
            desviacion_grados = math.degrees(desviacion_rad)
            
            # Frenamos exactamente a los 40 grados de apertura (Calibra aquí si necesitas más apertura)
            if desviacion_grados >= 40.0:
                self.get_logger().info(f"Apertura completada ({desviacion_grados:.1f} grados). Frenando...")
                self.estado = 'FRENANDO'
                self.tiempo_inicio_estado = time.time()

        elif self.estado == 'FRENANDO':
            cmd.dir_dc = 0
            cmd.speed_dc = 0
            cmd.dir_servo = 1500 
            cmd.turn_signals = 3 
            cmd.stop_lights = 1  
            
            # Mantenemos este pequeño cronómetro solo para que la física del carro se asiente
            if time.time() - self.tiempo_inicio_estado > 1.0:
                self.estado = 'REVERSA_CURVA'
                self.tiempo_inicio_estado = time.time()
                self.get_logger().info("Metiendo la cola. Esperando ángulo de 85 grados...")
                
        elif self.estado == 'REVERSA_CURVA':
            cmd.dir_dc = 2       
            cmd.speed_dc = 40    
            cmd.dir_servo = 1740 # En reversa, las llantas hacia la izquierda meten la cola
            cmd.turn_signals = 3 
            cmd.stop_lights = 0
            
            # --- CIERRE DE LAZO: ODOMETRÍA ---
            desviacion_rad = abs(self.diferencia_angular(self.yaw_actual, self.yaw_calle))
            desviacion_grados = math.degrees(desviacion_rad)
            
            # Queremos quedar casi perpendiculares a la calle (85 grados)
            if desviacion_grados >= 85.0:
                self.get_logger().info(f"Ángulo de encuadre alcanzado ({desviacion_grados:.1f} grados).")
                self.estado = 'REVERSA_RECTA'
                
        elif self.estado == 'REVERSA_RECTA':
            cmd.dir_dc = 2       
            cmd.speed_dc = 35
            cmd.dir_servo = 1500 
            cmd.turn_signals = 3 
            cmd.stop_lights = 0
            
            # 1. Radar Trasero
            if 0.0 < self.distancia_atras <= 0.12:
                self.get_logger().info(f"¡Radar trasero! Freno a {self.distancia_atras:.2f}m.")
                self.estado = 'ESTACIONADO'
                
            # 2. Navegación Relativa (Sándwich)
            elif 0.0 < self.distancia_izq < 0.25 and 0.0 < self.distancia_derecha < 0.25:
                if self.tiempo_sandwich == 0.0:
                    self.tiempo_sandwich = time.time()
                    
                elif time.time() - self.tiempo_sandwich > 1.2:
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