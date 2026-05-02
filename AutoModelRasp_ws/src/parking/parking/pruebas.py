#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
import sys

class CalibradorMotores(Node):
    def __init__(self):
        super().__init__('calibrador_motores')
        
        self.cmd_pub = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.encoder_callback, 10)
        
        # Estado del calibrador
        self.fase = 'DEADBAND' # Fases: DEADBAND -> ODOMETRIA
        
        # Variables para Zona Muerta
        self.pwm_test = 0
        self.umbral_movimiento = 3 # Ticks mínimos para considerar que ya se mueve
        
        # Variables para Odometría
        self.ticks_acumulados_m1 = 0
        self.ticks_acumulados_m2 = 0
        
        self.timer = self.create_timer(0.2, self.rutina_calibracion) # Aumenta PWM cada 200ms
        
        self.get_logger().info('=== INICIANDO HERRAMIENTA DE CALIBRACIÓN ===')
        self.get_logger().info('Fase 1: Buscando Zona Muerta (Deadband)... ¡Despeja el área frontal!')

    def encoder_callback(self, msg):
        if self.fase == 'DEADBAND':
            # Si detectamos que cualquier motor se movió más allá del ruido del sensor
            if msg.vel_m1 > self.umbral_movimiento or msg.vel_m2 > self.umbral_movimiento:
                self.fase = 'TRANSICION'
                self.get_logger().info('=========================================')
                self.get_logger().info(f'>> ¡MOVIMIENTO DETECTADO! <<')
                self.get_logger().info(f'>> TU PWM MÍNIMO (Zona Muerta) ES: {self.pwm_test} <<')
                self.get_logger().info('=========================================')
                
        elif self.fase == 'ODOMETRIA':
            # Acumulamos ticks cuando lo empujas a mano
            if msg.dir_m1 == 1: self.ticks_acumulados_m1 += msg.vel_m1
            elif msg.dir_m1 == 2: self.ticks_acumulados_m1 -= msg.vel_m1
            
            if msg.dir_m2 == 1: self.ticks_acumulados_m2 += msg.vel_m2
            elif msg.dir_m2 == 2: self.ticks_acumulados_m2 -= msg.vel_m2
            
            promedio = (abs(self.ticks_acumulados_m1) + abs(self.ticks_acumulados_m2)) // 2
            
            # Imprime en la misma línea para no saturar la terminal (Carriage Return)
            sys.stdout.write(f'\rTicks Promedio Acumulados: {promedio} | Presiona Ctrl+C al terminar')
            sys.stdout.flush()

    def rutina_calibracion(self):
        comando = MotorCommand()
        comando.dir_servo = 1500 # Volante derecho
        comando.stop_lights = 0
        comando.turn_signals = 0
        
        if self.fase == 'DEADBAND':
            self.pwm_test += 1 # Rampa de aceleración suave
            comando.dir_dc = 1
            comando.speed_dc = self.pwm_test
            self.cmd_pub.publish(comando)
            self.get_logger().info(f'Probando PWM: {self.pwm_test}%...')
            
            if self.pwm_test > 90:
                self.get_logger().warn('Llegamos al 90% y no hay movimiento. Revisa conexiones.')
                self.fase = 'FIN'
                
        elif self.fase == 'TRANSICION':
            # Apagar motores
            comando.dir_dc = 0
            comando.speed_dc = 0
            comando.stop_lights = 1
            self.cmd_pub.publish(comando)
            
            self.get_logger().info('Fase 2: Calibración de Odometría.')
            self.get_logger().info('-> Empuja el carro a mano EXACTAMENTE 1 METRO (100 cm).')
            self.get_logger().info('-> O dale EXACTAMENTE 1 VUELTA a la llanta trasera.')
            self.get_logger().info('Leyendo encoders... (Gira la llanta ahora)')
            self.fase = 'ODOMETRIA'
            
        elif self.fase == 'ODOMETRIA':
            # Mantener el carro en alto total lógico para que el STM32 solo reporte ticks
            comando.dir_dc = 0
            comando.speed_dc = 0
            self.cmd_pub.publish(comando)

def main(args=None):
    rclpy.init(args=args)
    node = CalibradorMotores()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n')
        node.get_logger().info('=== CALIBRACIÓN FINALIZADA ===')
        node.get_logger().info('Anota estos valores para tu máquina de estados:')
        node.get_logger().info(f'- PWM_MIN: {node.pwm_test}')
        node.get_logger().info(f'- TICKS FINALES: {(abs(node.ticks_acumulados_m1) + abs(node.ticks_acumulados_m2)) // 2}')
    finally:
        # Freno de seguridad al salir
        pub = node.cmd_pub
        msg = MotorCommand()
        msg.dir_dc = 0
        msg.speed_dc = 0
        msg.dir_servo = 1500
        pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()