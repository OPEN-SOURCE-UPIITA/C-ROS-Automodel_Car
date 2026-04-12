#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from motor_msgs.msg import MotorCommand
import numpy as np


class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        # -------- PARÁMETROS DINÁMICOS PARA RQT --------
        
        # 0. Botón de Seguridad (Start / Stop)
        # Aparecerá como una casilla (checkbox) en RQT. Inicia en False (Detenido).
        self.declare_parameter('habilitar_conduccion', False)

        # 1. Control de Velocidad y Dirección
        self.declare_parameter('velocidad_dc', 80)           # Velocidad crucero máxima en rectas
        self.declare_parameter('velocidad_minima', 60)       # Piso absoluto para evitar que el motor se congele
        self.declare_parameter('factor_reduccion_vel', 20)   # Cuánto PWM se resta de la vel_base al tomar una curva
        self.declare_parameter('kp_servo', 198.0)            # Fuerza de giro (Proporcional)
        self.declare_parameter('kd_servo', 50.0)             # Anticipación (Derivativo)
        self.declare_parameter('max_step_servo', 25)         # Suavizado de movimiento del volante

        # 2. Control de Señales (Frenado)
        self.declare_parameter('distancia_paro_min', 0.15)   # Distancia mínima para activar STOP (m)
        self.declare_parameter('distancia_paro_max', 0.40)   # Distancia máxima para activar STOP (m)
        self.declare_parameter('tiempo_detenido', 5.0)       # Segundos que el auto debe quedarse quieto
        self.declare_parameter('tiempo_ignorar', 4.0)        # Segundos de "cooldown" para arrancar sin volver a frenar

        # Variables para Servo y PD
        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        self.error_previo = 0.0
        self.last_time = self.get_clock().now()

        # Máquina de estados para Frenado
        self.distancia_senal = 999.0
        self.estado_vehiculo = "CONDUCIENDO" # Estados posibles: CONDUCIENDO, DETENIDO, IGNORANDO
        self.tiempo_cambio_estado = self.get_clock().now()

        # -------- SUBSCRIPCIONES Y PUBLICACIONES --------
        self.sub_error = self.create_subscription(
            Float32, '/steering_error', self.error_callback, 10)

        self.sub_distancia = self.create_subscription(
            Float32, '/vision/senal_stop/distancia_nube', self.distancia_callback, 10)

        self.pub_motor = self.create_publisher(
            MotorCommand, '/motor_command', 10)

        self.get_logger().info("Nodo Drive Carril (Modo Seguro: Esperando Habilitación en RQT)")

    def distancia_callback(self, msg):
        """Actualiza la memoria con la distancia reportada por la cámara para la señal de STOP."""
        self.distancia_senal = msg.data

    def evaluar_estado_frenado(self):
        """
        Máquina de estados independiente.
        Retorna: True si el motor debe detenerse por una señal, False si puede avanzar.
        """
        ahora = self.get_clock().now()
        dt_estado = (ahora - self.tiempo_cambio_estado).nanoseconds / 1e9

        d_min = self.get_parameter('distancia_paro_min').value
        d_max = self.get_parameter('distancia_paro_max').value
        t_paro = self.get_parameter('tiempo_detenido').value
        t_ign = self.get_parameter('tiempo_ignorar').value

        if self.estado_vehiculo == "CONDUCIENDO":
            if d_min <= self.distancia_senal <= d_max:
                self.estado_vehiculo = "DETENIDO"
                self.tiempo_cambio_estado = ahora
                self.get_logger().warn(f"¡SEÑAL DETECTADA a {self.distancia_senal:.2f}m! Frenando {t_paro}s.")
                return True
            return False

        elif self.estado_vehiculo == "DETENIDO":
            if dt_estado >= t_paro:
                self.estado_vehiculo = "IGNORANDO"
                self.tiempo_cambio_estado = ahora
                self.distancia_senal = 999.0 # Reseteo de memoria
                self.get_logger().info("Tiempo de STOP cumplido. Arrancando (Modo Ignorar)...")
                return False
            return True

        elif self.estado_vehiculo == "IGNORANDO":
            if dt_estado >= t_ign:
                self.estado_vehiculo = "CONDUCIENDO"
                self.distancia_senal = 999.0
                self.get_logger().info("Fin del modo ignorar. Detección normal restaurada.")
            return False

    def error_callback(self, msg):
        ahora = self.get_clock().now()
        
        # --- 0. BOTÓN START/STOP (Interruptor de Seguridad) ---
        modo_activo = self.get_parameter('habilitar_conduccion').value
        
        if not modo_activo:
            # Si el botón está apagado (False), forzamos paro inmediato
            cmd_stop = MotorCommand()
            cmd_stop.dir_dc = 0
            cmd_stop.speed_dc = 0
            cmd_stop.dir_servo = int(self.last_servo_value) # Mantiene la posición de las llantas
            cmd_stop.turn_signals = 0
            self.pub_motor.publish(cmd_stop)
            
            # Reseteamos el reloj para que no haya un salto brusco en la Derivada al encenderlo
            self.last_time = ahora
            return

        # --- 1. TIEMPO (dt) PARA DERIVADA ---
        dt = (ahora - self.last_time).nanoseconds / 1e9
        self.last_time = ahora
        if dt <= 0: dt = 0.033 

        error_actual = msg.data

        # --- 2. LECTURA DE PARÁMETROS (RQT) ---
        vel_base = self.get_parameter('velocidad_dc').value
        vel_min = self.get_parameter('velocidad_minima').value
        kp = self.get_parameter('kp_servo').value
        kd = self.get_parameter('kd_servo').value
        base_step = self.get_parameter('max_step_servo').value
        f_red = self.get_parameter('factor_reduccion_vel').value

        # --- 3. CONTROLADOR PD PARA DIRECCIÓN ---
        P = error_actual * kp
        D = kd * (error_actual - self.error_previo) / dt
        self.error_previo = error_actual

        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # Suavizado de giro (Slew Rate) para no golpear la dirección mecánicamente
        max_step = base_step + int(abs(error_actual) * 20)
        diff = target_servo - self.last_servo_value
        
        if abs(diff) > max_step:
            nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
        else:
            nuevo_servo = target_servo
            
        self.last_servo_value = nuevo_servo

        # --- 4. CÁLCULO DE VELOCIDAD ADAPTATIVA (ANTI-ZONA MUERTA) ---
        # Reduce vel_base según el error, garantizando que NUNCA baje de vel_min
        speed_sugerida = vel_base - (abs(error_actual) * f_red)
        speed_calculada = int(max(vel_min, min(speed_sugerida, vel_base)))

        # --- 5. FUSIÓN CON MÁQUINA DE FRENADO ---
        if self.evaluar_estado_frenado():
            speed_dc = 0 
        else:
            speed_dc = speed_calculada

        # --- 6. COMANDO FINAL A MOTORES ---
        cmd = MotorCommand()
        cmd.dir_dc = 1
        cmd.speed_dc = speed_dc
        cmd.dir_servo = int(nuevo_servo)
        
        # Luces direccionales
        if error_actual > 0.3:
            cmd.turn_signals = 1 
        elif error_actual < -0.3:
            cmd.turn_signals = 2 
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
        # Freno de emergencia si se cierra el programa en la terminal
        stop = MotorCommand()
        stop.dir_dc = 0
        stop.speed_dc = 0
        stop.dir_servo = 1500
        stop.turn_signals = 0
        node.pub_motor.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
