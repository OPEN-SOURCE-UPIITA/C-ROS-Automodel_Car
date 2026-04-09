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
        self.declare_parameter('max_step_servo', 15)
        self.declare_parameter('factor_reduccion_vel', 0.5)

        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro

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

        self.get_logger().info("Drive Carril PRO (agresivo en curvas) listo")

    def error_callback(self, msg):
        error = msg.data

        # -------- PARÁMETROS --------
        vel_base = self.get_parameter('velocidad_dc').value
        kp = self.get_parameter('kp_servo').value
        base_step = self.get_parameter('max_step_servo').value
        factor_red = self.get_parameter('factor_reduccion_vel').value

        # -------- CONTROL SERVO --------
        # Ganancia base
        target_servo = self.servo_centro + (error * kp) ##aqui cambiamos el signo 

        # Limitar rango físico
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # -------- SLEW RATE DINÁMICO --------
        # Más error = más velocidad de giro
        max_step = base_step + int(abs(error) * 50)

        diff = target_servo - self.last_servo_value

        # -------- MODO AGRESIVO EN CURVAS --------
        if abs(error) > 0.7:
            nuevo_servo = 1740 if error > 0 else 1110
        else:
            target_servo = self.servo_centro + (error * kp)
            target_servo = int(np.clip(target_servo, 1110, 1740))

            diff = target_servo - self.last_servo_value
        if abs(diff) > max_step:
            nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
        else:
            nuevo_servo = target_servo

        # Guardar estado
        self.last_servo_value = nuevo_servo

        # -------- VELOCIDAD ADAPTATIVA --------
        reduccion = 1.0 - (abs(error) * factor_red)
        reduccion = max(0.3, reduccion)  # nunca bajar de 30%

        speed_dc = int(vel_base * reduccion)

        # -------- DIRECCIÓN --------
        dir_dc = 1

        # -------- LUCES --------
        turn = 0
        if error > 0.3:
            turn = 1
        elif error < -0.3:
            turn = 2

        # -------- PUBLICAR --------
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
        stop_cmd = MotorCommand()
        stop_cmd.dir_dc = 0
        stop_cmd.speed_dc = 0
        stop_cmd.dir_servo = 1500
        node.pub_motor.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
