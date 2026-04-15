#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os

class LidarAtras(Node):
    def __init__(self):
        super().__init__('lidar_atras_test')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0: return

        # Dividimos el escaneo en sectores para identificar dónde está la "atrás" real
        # En tu LIDAR, el índice 0 suele ser un extremo y el final el otro.
        
        sector_inicio = msg.ranges[0:20]          # Los primeros rayos
        sector_mitad  = msg.ranges[n//2-10 : n//2+10] # El centro del arreglo
        sector_final  = msg.ranges[n-20 : n]      # Los últimos rayos

        def limpiar(lista):
            return [round(d, 3) for d in lista if 0.02 < d < 12.0]

        # Limpiamos ruido e infinitos
        ini = limpiar(sector_inicio)
        mit = limpiar(sector_mitad)
        fin = limpiar(sector_final)

        # Impresión limpia en terminal
        os.system('clear')
        print("="*50)
        print(" 👀 MONITOR DE DISTANCIAS TRASERAS")
        print("="*50)
        print(f"Total de rayos en el sensor: {n}")
        print("-" * 50)
        print(f"ZONA INICIAL (Índice 0-20):  {sum(ini)/len(ini) if ini else 'INF':.3f} m")
        print(f"ZONA MEDIA   (Índice {n//2}): {sum(mit)/len(mit) if mit else 'INF':.3f} m")
        print(f"ZONA FINAL   (Índice {n-20}-max): {sum(fin)/len(fin) if fin else 'INF':.3f} m")
        print("-" * 50)
        print("\nLectura cruda central (para ver si detectas chasis):")
        print(mit)
        print("\n👉 Mueve tu mano atrás del carro y mira qué zona cambia.")
        print("Si ves valores de 0.05 a 0.08 constantes, ¡es tu chasis!")

def main():
    rclpy.init()
    rclpy.spin(LidarAtras())
    rclpy.shutdown()

if __name__ == '__main__':
    main()