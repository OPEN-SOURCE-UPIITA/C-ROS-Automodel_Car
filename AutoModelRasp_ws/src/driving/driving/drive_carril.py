#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from motor_msgs.msg import MotorCommand
import numpy as np

class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        self.declare_parameter('velocidad_dc', 40)           
        self.declare_parameter('kp_servo', 250.0)            
        self.declare_parameter('kd_servo', 50.0)             
        self.declare_parameter('max_step_servo', 30)         
        self.declare_parameter('factor_reduccion_vel', 0.8)  

        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        
        self.error_previo = 0.0
        self.last_time = self.get_clock().now()

        self.sub_error = self.create_subscription(Float32, '/steering_error', self.error_callback, 10)
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)

    def error_callback(self, msg):
        error_actual = msg.data
        
        # --- CÁLCULO DE TIEMPO REAL (dt) ---
        ahora = self.get_clock().now()
        dt = (ahora - self.last_time).nanoseconds / 1e9
        self.last_time = ahora
        if dt <= 0: dt = 0.033 

        vel_base = self.get_parameter('velocidad_dc').value
        kp = self.get_parameter('kp_servo').value
        kd = self.get_parameter('kd_servo').value
        base_step = self.get_parameter('max_step_servo').value
        f_red = self.get_parameter('factor_reduccion_vel').value

        # --- CONTROL PD ROBUSTO ---
        P = error_actual * kp
        D = kd * (error_actual - self.error_previo) / dt
        self.error_previo = error_actual

        # Inversión de signo incluida: "-" corrige el giro físico del servo
        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # --- SLEW RATE ---
        max_step = base_step + int(abs(error_actual) * 20)
        diff = target_servo - self.last_servo_value
        
        if abs(diff) > max_step:
            nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
        else: nuevo_servo = target_servo
            
        self.last_servo_value = nuevo_servo

        # --- VELOCIDAD ADAPTATIVA CUADRÁTICA ---
        # Penaliza más los errores grandes (curvas cerradas)
        reduccion = 1.0 - (abs(error_actual)**2 * f_red)
        speed_dc = int(vel_base * max(0.2, reduccion))

        cmd = MotorCommand()
        cmd.dir_dc = 1
        cmd.speed_dc = speed_dc
        cmd.dir_servo = int(nuevo_servo)
        
        if error_actual > 0.3: cmd.turn_signals = 1 
        elif error_actual < -0.3: cmd.turn_signals = 2 
        else: cmd.turn_signals = 0

        self.pub_motor.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DriveCarrilNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        stop = MotorCommand()
        stop.dir_dc, stop.speed_dc, stop.dir_servo = 0, 0, 1500
        node.pub_motor.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()