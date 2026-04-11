import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

from motor_msgs.msg import EncoderData

class EncoderAdapter(Node):
    def __init__(self):
        super().__init__('encoder_adapter')
        
        self.sub_odom = self.create_subscription(Odometry, '/model/carro/odometry', self.odom_callback, 10)
        
        self.pub_enc = self.create_publisher(EncoderData, '/encoder_data', 10)
        
        # --- PARÁMETROS DE CALIBRACIÓN FÍSICA ---
        # 1. ¿Cuántos ticks da tu encoder real cuando el carro avanza 1 metro?
        self.ticks_por_metro = 1000.0 
        
        # 2. ¿A qué frecuencia lee? (10Hz = ciclos de 0.1s)
        self.frecuencia_ciclo = 10.0  
        
        # 3. Distancia entre las llantas traseras en metros
        self.distancia_eje = 0.15 
        
        self.get_logger().info("Adaptador Sim-to-Real: Odometría a Encoders Iniciado.")

    def odom_callback(self, msg):
        v_x = msg.twist.twist.linear.x       # Velocidad en m/s
        omega_z = msg.twist.twist.angular.z  # Giro en radianes/s

        v_left = v_x - (omega_z * self.distancia_eje / 2.0)
        v_right = v_x + (omega_z * self.distancia_eje / 2.0)

        def calcular_encoder(v_rueda):
            ticks_por_seg = v_rueda * self.ticks_por_metro
            
            ticks_ciclo = ticks_por_seg / self.frecuencia_ciclo
            
            vel_abs = int(abs(ticks_ciclo))
            
            if v_rueda > 0.01:
                dir_val = 1
            elif v_rueda < -0.01:
                dir_val = 2
            else:
                dir_val = 0
                vel_abs = 0
                
            return vel_abs, dir_val

        vel_m1, dir_m1 = calcular_encoder(v_left)
        vel_m2, dir_m2 = calcular_encoder(v_right)

        enc_msg = EncoderData()
        enc_msg.vel_m1 = vel_m1
        enc_msg.dir_m1 = dir_m1
        enc_msg.vel_m2 = vel_m2
        enc_msg.dir_m2 = dir_m2

        self.pub_enc.publish(enc_msg)

def main():
    rclpy.init()
    rclpy.spin(EncoderAdapter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()