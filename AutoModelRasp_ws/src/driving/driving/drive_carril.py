#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from motor_msgs.msg import MotorCommand  # Importamos tu mensaje personalizado

class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        # --- PARÁMETROS CONFIGURABLES ---
        # Velocidad base (0-100)
        self.declare_parameter('velocidad_crucero', 100)
        # Ganancia de giro (qué tanto responde al error)
        self.declare_parameter('kp_servo', 315.0) # (1740-1110)/2 = 315
        # Centro del servo (según tu código de STM32)
        self.servo_centro = 1500

        # --- SUSCRIPCIONES Y PUBLICADORES ---
        self.sub_error = self.create_subscription(
            Float32,
            '/steering_error',
            self.error_callback,
            10)
        
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)

        self.get_logger().info("Nodo Drive Carril Iniciado - Esperando errores de visión...")

    def error_callback(self, msg):
        error = msg.data  # Valor entre -1.0 y 1.0
        
        # Obtener parámetros actuales
        vel_base = self.get_parameter('velocidad_crucero').value
        kp = self.get_parameter('kp_servo').value

        # --- LÓGICA DE CONTROL ---
        
        # 1. Dirección (Servo)
        # Si error es positivo (derecha), sumamos al centro. 
        # Si es negativo (izquierda), restamos.
        nuevo_servo = self.servo_centro + (error * kp)
        
        # Limitamos por seguridad (aunque el STM32 también lo hace)
        nuevo_servo = int(max(1110, min(1740, nuevo_servo)))

        # 2. Motores DC
        # Por ahora mantendremos velocidad constante si hay carril
        dir_dc = 1 # Adelante
        speed_dc = vel_base

        # Opcional: Reducir velocidad si el giro es muy brusco
        if abs(error) > 0.6:
            speed_dc = int(vel_base * 0.7)

        # 3. Luces (Turn Signals)
        turn = 0
        if error > 0.3: turn = 1    # Derecha
        elif error < -0.3: turn = 2  # Izquierda

        # --- PUBLICAR COMANDO ---
        cmd = MotorCommand()
        cmd.dir_dc = dir_dc
        cmd.speed_dc = speed_dc
        cmd.dir_servo = nuevo_servo
        
        # Luces
        cmd.stop_lights = 0
        cmd.turn_signals = turn

        self.pub_motor.publish(cmd)
        
        # Log para debug (opcional, puedes comentarlo después)
        # self.get_logger().info(f"Error: {error:.2f} -> Servo: {nuevo_servo} | Vel: {speed_dc}")

def main(args=None):
    rclpy.init(args=args)
    node = DriveCarrilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al detener, mandamos comando de stop por seguridad
        stop_cmd = MotorCommand()
        stop_cmd.dir_dc = 0
        stop_cmd.speed_dc = 0
        stop_cmd.dir_servo = 1500
        node.pub_motor.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
