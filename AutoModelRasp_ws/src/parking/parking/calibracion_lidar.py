import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class CalibradorLidar(Node):
    def __init__(self):
        super().__init__('calibrador_lidar')
        
        # Suscripción directa al tópico del Lidar
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        # Ángulos estándar a monitorear (en Radianes)
        self.angulos = {
            "A (0.00 rad)": 0.0,
            "B (1.57 rad)": 1.5708,  # 90 grados
            "C (3.14 rad)": 3.1416,  # 180 grados
            "D (4.71 rad)": 4.7124   # 270 grados
        }
        
        self.lecturas = {k: 0.0 for k in self.angulos.keys()}
        
        # Timer para imprimir limpio en consola (cada medio segundo)
        self.timer = self.create_timer(0.5, self.imprimir_brujula)
        self.get_logger().info("📡 INICIANDO RADAR DE CALIBRACIÓN LIDAR 📡")
        self.get_logger().info("Pon un objeto cerca de tu carro en un lado a la vez y mira qué letra cambia.")

    def scan_cb(self, msg):
        num_rayos = len(msg.ranges)
        if num_rayos == 0:
            return

        def medir_angulo(angulo_objetivo):
            # Encontrar el índice del rayo que corresponde a ese ángulo
            diff = math.atan2(math.sin(angulo_objetivo - msg.angle_min), math.cos(angulo_objetivo - msg.angle_min))
            if diff < 0:
                diff += 2.0 * math.pi
                
            idx = int(diff / msg.angle_increment)
            if idx >= num_rayos: 
                idx = num_rayos - 1
            
            # Tomamos una pequeña muestra de 5 rayos alrededor para no fallar por un milímetro
            rayos = msg.ranges[max(0, idx-2):min(num_rayos, idx+2)]
            
            # Filtramos basura (infinitos, ceros absolutos, etc)
            validos = [d for d in rayos if 0.05 < d < 12.0 and not math.isinf(d) and not math.isnan(d)]
            
            # Devolvemos el mínimo de esa pequeña zona
            return min(validos) if validos else 99.99

        # Actualizamos el diccionario con las lecturas en tiempo real
        for nombre, rads in self.angulos.items():
            self.lecturas[nombre] = medir_angulo(rads)

    def imprimir_brujula(self):
        # Limpiamos un poco la pantalla (funciona en la mayoría de terminales Linux)
        print("\033[H\033[J", end="")
        
        print("=========================================")
        print("      BRÚJULA DE CALIBRACIÓN LIDAR       ")
        print("=========================================\n")
        
        print(f"            [ FRENTE DEL CARRO ]           ")
        print(f"                    ^                      ")
        print(f"                    |                      ")
        
        # Formateo para que se vea como una cruz
        val_A = f"{self.lecturas['A (0.00 rad)']:.2f}m"
        val_B = f"{self.lecturas['B (1.57 rad)']:.2f}m"
        val_C = f"{self.lecturas['C (3.14 rad)']:.2f}m"
        val_D = f"{self.lecturas['D (4.71 rad)']:.2f}m"

        print(f"               A (0.00 rad)                ")
        print(f"                 [{val_A}]                 ")
        print(f"                    |                      ")
        print(f" B (1.57) [{val_B}] -+- [{val_D}] D (4.71) ")
        print(f"                    |                      ")
        print(f"                 [{val_C}]                 ")
        print(f"               C (3.14 rad)                ")
        print("\n=========================================")
        print("Identifica cuál es Derecha y cuál es Atrás")
        print("para poner esos radianes en smart_parking.py")

def main():
    rclpy.init()
    try:
        rclpy.spin(CalibradorLidar())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()