#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from motor_msgs.msg import MotorCommand
import numpy as np

class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        # -------- PARÁMETROS RQT --------
        self.declare_parameter('velocidad_dc', 30)
        self.declare_parameter('kp_servo', 200.0)
        self.declare_parameter('kd_servo', 100.0)  # NUEVO: Ganancia Derivativa
        self.declare_parameter('max_step_servo', 15)
        self.declare_parameter('factor_reduccion_vel', 0.5)

        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        self.last_error = 0.0  # NUEVO: Memoria para la derivada

        # -------- ROS --------
        self.sub_error = self.create_subscription(
            Float32,
            '/steering_error',
            self.error_callback,
            10)

        self.pub_motor = self.create_publisher(
            MotorCommand,
            '/motor_command',
            10)

        self.get_logger().info("Controlador PD (Tracción + Dirección) listo")

    def error_callback(self, msg):
        # INVERTIMOS EL ERROR AQUI MISMO PARA CORREGIR EL SERVO FISICO
        error = -msg.data

        # -------- PARÁMETROS --------
        vel_base = self.get_parameter('velocidad_dc').value
        kp = self.get_parameter('kp_servo').value
        kd = self.get_parameter('kd_servo').value
        base_step = self.get_parameter('max_step_servo').value
        factor_red = self.get_parameter('factor_reduccion_vel').value

        # -------- 1. CÁLCULO PD --------
        # Calculamos qué tan rápido está cambiando el error
        derivada = error - self.last_error
        
        # Fórmula PD: Centro + (Proporcional) + (Derivativo)
        # Nota: Ajusta el signo de 'error * kp' si el coche gira al revés
        target_servo = self.servo_centro + (error * kp) + (derivada * kd)
        
        # Actualizamos la memoria para el siguiente ciclo
        self.last_error = error

        # Limitar al rango físico del servo
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # -------- 2. SLEW RATE DINÁMICO --------
        # Evita que el servo dé tirones bruscos que rompan la tracción
        max_step = base_step + int(abs(error) * 50)
        diff = target_servo - self.last_servo_value

        # -------- 3. MODO AGRESIVO EN CURVAS CERRADAS --------
        if abs(error) > 0.7:
            # Si el error es masivo, mete todo el volante
            nuevo_servo = 1740 if error > 0 else 1110
        else:
            if abs(diff) > max_step:
                nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
            else:
                nuevo_servo = target_servo

        # Guardar estado físico del servo
        self.last_servo_value = nuevo_servo

        # -------- 4. VELOCIDAD ADAPTATIVA (FRENO EN CURVAS) --------
        reduccion = 1.0 - (abs(error) * factor_red)
        reduccion = max(0.3, reduccion)  # Nunca bajar de 30% de la vel_base

        speed_dc = int(vel_base * reduccion)
        dir_dc = 1 # 1 = Adelante

        # -------- 5. DIRECCIONALES --------
        turn = 0
        if error > 0.3:
            turn = 1
        elif error < -0.3:
            turn = 2

        # -------- PUBLICAR COMANDO AL STM32 --------
        cmd = MotorCommand()
        cmd.dir_dc = dir_dc
        cmd.speed_dc = speed_dc
        cmd.dir_servo = int(np.clip(nuevo_servo, 1110, 1740))
        cmd.stop_lights = 0
        cmd.turn_signals = turn

        self.pub_motor.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DriveCarrilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Freno de emergencia al apagar el nodo
        stop_cmd = MotorCommand()
        stop_cmd.dir_dc = 0
        stop_cmd.speed_dc = 0
        stop_cmd.dir_servo = 1500
        node.pub_motor.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()