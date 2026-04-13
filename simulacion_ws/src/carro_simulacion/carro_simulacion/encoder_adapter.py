import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from motor_msgs.msg import EncoderData
import time

class EncoderAdapter(Node):
    def __init__(self):
        super().__init__('encoder_adapter')
        
        self.sub_odom = self.create_subscription(Odometry, '/model/carro/odometry', self.odom_callback, 10)
        self.pub_enc = self.create_publisher(EncoderData, '/encoder_data', 10)
        
        # Parámetros físicos
        self.ticks_por_metro = 2060.0 
        self.ancho_via = 0.15 # Ajustado a tu URDF
        
        # Memoria de tiempo para cálculo dinámico
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("Adaptador Sim-to-Real: Sincronización Temporal Activa.")

    def odom_callback(self, msg):
        # 1. Calcular el Delta de Tiempo (dt) real entre mensajes
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Evitar saltos de tiempo en el primer mensaje o errores de simulación
        if dt <= 0 or dt > 0.5:
            self.last_time = current_time
            return

        # 2. Obtener velocidades de Gazebo
        v_x = msg.twist.twist.linear.x 
        omega_z = msg.twist.twist.angular.z 

        # 3. Cinemática Diferencial para sacar velocidad de cada rueda (m/s)
        v_left = v_x - (omega_z * self.ancho_via / 2.0)
        v_right = v_x + (omega_z * self.ancho_via / 2.0)

        # 4. Convertir Velocidad Lineal a DISTANCIA recorrida en este micro-ciclo (metros)
        dist_left = v_left * dt
        dist_right = v_right * dt

        # 5. Convertir DISTANCIA a TICKS (usando flotantes para no perder precisión)
        ticks_l = dist_left * self.ticks_por_metro
        ticks_r = dist_right * self.ticks_por_metro

        # 6. Publicar (Aquí mandamos el delta de ticks que el parking sumará)
        enc_msg = EncoderData()
        # Mandamos el valor absoluto como vel_m1 (tu parking usa abs())
        # Multiplicamos por 10 solo si tu parking espera 'ticks por décima de segundo'
        # Pero lo ideal es que mandes los ticks reales del ciclo:
        enc_msg.vel_m1 = int(ticks_l) 
        enc_msg.vel_m2 = int(ticks_r)
        
        # Direcciones (opcional, tu parking usa el comando_dir del motor)
        enc_msg.dir_m1 = 1 if ticks_l >= 0 else 2
        enc_msg.dir_m2 = 1 if ticks_r >= 0 else 2

        self.pub_enc.publish(enc_msg)
        self.last_time = current_time

def main():
    rclpy.init()
    rclpy.spin(EncoderAdapter())
    rclpy.shutdown()