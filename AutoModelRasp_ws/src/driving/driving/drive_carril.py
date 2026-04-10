#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from motor_msgs.msg import MotorCommand
import numpy as np

class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        # -------- PARÁMETROS PARA RQT_RECONFIGURE --------
        self.declare_parameter('velocidad_dc', 40)           # Velocidad crucero en rectas
        self.declare_parameter('kp_servo', 250.0)            # Fuerza de giro (Proporcional)
        self.declare_parameter('kd_servo', 50.0)             # Amortiguación/Anticipación (Derivativo)
        self.declare_parameter('max_step_servo', 30)         # Suavizado de movimiento del volante
        self.declare_parameter('factor_reduccion_vel', 0.8)  # 0.0 = No frena, 1.0 = Frena al máximo

        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        
        # Variables de estado para el control PD
        self.error_previo = 0.0
        self.last_time = self.get_clock().now()

        # -------- SUBSCRIPCIONES Y PUBLICACIONES --------
        self.sub_error = self.create_subscription(
            Float32,
            '/steering_error',
            self.error_callback,
            10)

        self.pub_motor = self.create_publisher(
            MotorCommand,
            '/motor_command',
            10)

        self.get_logger().info("Nodo Control PD + Velocidad Adaptativa (RQT) Iniciado")

    def error_callback(self, msg):
        error_actual = msg.data
        
        # --- CÁLCULO DE TIEMPO (dt) ---
        ahora = self.get_clock().now()
        dt = (ahora - self.last_time).nanoseconds / 1e9
        self.last_time = ahora
        
        # Evitar división por cero o saltos temporales extraños
        if dt <= 0: dt = 0.033 

        # --- LEER PARÁMETROS DESDE RQT ---
        vel_base = self.get_parameter('velocidad_dc').value
        kp = self.get_parameter('kp_servo').value
        kd = self.get_parameter('kd_servo').value
        base_step = self.get_parameter('max_step_servo').value
        f_red = self.get_parameter('factor_reduccion_vel').value

        # --- CÁLCULO CONTROL PD ---
        
        # 1. Parte Proporcional (Reacción al error actual)
        P = error_actual * kp
        
        # 2. Parte Derivativa (Reacción al cambio del error / anticipación)
        # mide qué tan rápido nos alejamos o acercamos al centro
        D = kd * (error_actual - self.error_previo) / dt
        
        # Guardar error para el siguiente ciclo
        self.error_previo = error_actual

        # Posición final del servo (Centro - corrección)
        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # --- SUAVIZADO DE GIRO (SLEW RATE) ---
        # Si la curva es muy cerrada, permitimos que el servo se mueva un poco más rápido
        max_step = base_step + int(abs(error_actual) * 20)
        diff = target_servo - self.last_servo_value
        
        if abs(diff) > max_step:
            nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
        else:
            nuevo_servo = target_servo
            
        self.last_servo_value = nuevo_servo

        # --- LÓGICA DE VELOCIDAD ADAPTATIVA ---
        reduccion = 1.0 - (abs(error_actual)**2 * f_red)
        speed_dc = int(vel_base * max(0.2, reduccion))

        # --- PUBLICACIÓN DEL COMANDO ---
        cmd = MotorCommand()
        cmd.dir_dc = 1
        cmd.speed_dc = speed_dc
        cmd.dir_servo = int(nuevo_servo)
        
        # Luces de giro
        if error_actual > 0.3:
            cmd.turn_signals = 1 # Derecha
        elif error_actual < -0.3:
            cmd.turn_signals = 2 # Izquierda
        else:
            cmd.turn_signals = 0

        self.pub_motor.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DriveCarrilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = MotorCommand()
        stop.dir_dc = 0
        stop.speed_dc = 0
        stop.dir_servo = 1500
        node.pub_motor.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()