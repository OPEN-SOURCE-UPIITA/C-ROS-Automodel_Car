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
        # Parámetros de Carril
        self.declare_parameter('velocidad_dc', 0)          # Velocidad crucero en rectas
        self.declare_parameter('kp_servo', 198.0)            # Fuerza de giro (Proporcional)
        self.declare_parameter('kd_servo', 50.0)             # Amortiguación/Anticipación (Derivativo)
        self.declare_parameter('max_step_servo', 25)         # Suavizado de movimiento del volante
        self.declare_parameter('factor_reduccion_vel', 5)  # 0.0 = No frena, 1.0 = Frena al máximo
        
        # Parámetros de Frenado (NUEVOS)
        self.declare_parameter('distancia_paro', 0.40)       # Distancia a la que se activa el freno (metros)
        self.declare_parameter('tiempo_paro', 5.0)           # Segundos que el auto debe quedarse quieto
        self.declare_parameter('tiempo_cooldown', 4.0)       # Segundos de inmunidad a señales tras arrancar

        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        
        # Variables de estado para el control PD
        self.error_previo = 0.0
        self.last_time = self.get_clock().now()

        # Variables de Máquina de Estados (Frenado)
        self.estado_vehiculo = "CONDUCIENDO" # "CONDUCIENDO" o "PARADO"
        self.tiempo_fin_paro = 0.0
        self.tiempo_ignorar_senal = 0.0

        # -------- SUBSCRIPCIONES Y PUBLICACIONES --------
        self.sub_error = self.create_subscription(
            Float32, '/steering_error', self.error_callback, 10)
            
        # NUEVA SUSCRIPCIÓN AL SENSOR DE DISTANCIA DE LA SEÑAL
        self.sub_distancia = self.create_subscription(
            Float32, '/vision/senal_stop/distancia_nube', self.distancia_callback, 10)

        self.pub_motor = self.create_publisher(
            MotorCommand, '/motor_command', 10)

        self.get_logger().info("Nodo Drive Control + Frenado Modular Iniciado")

    # ==========================================================
    # MÓDULO DE FRENADO
    # ==========================================================
    def distancia_callback(self, msg):
        """Recibe la distancia de la señal y activa la secuencia de frenado si es necesario."""
        dist = msg.data
        dist_paro = self.get_parameter('distancia_paro').value
        
        if 0.0 < dist <= dist_paro:
            ahora = self.get_clock().now().nanoseconds / 1e9
            
            # Solo frenamos si estábamos conduciendo y ya pasamos el tiempo de "cooldown"
            if self.estado_vehiculo == "CONDUCIENDO" and ahora > self.tiempo_ignorar_senal:
                self.iniciar_frenado(ahora)

    def iniciar_frenado(self, tiempo_actual):
        """Cambia el estado del vehículo y calcula cuándo debe volver a arrancar."""
        tiempo_espera = self.get_parameter('tiempo_paro').value
        self.estado_vehiculo = "PARADO"
        self.tiempo_fin_paro = tiempo_actual + tiempo_espera
        self.get_logger().warn(f"¡SEÑAL DE STOP! Deteniendo el auto por {tiempo_espera} segundos...")

    def verificar_y_aplicar_freno(self, tiempo_actual):
        """
        Función modular de control de estado.
        Retorna True si el auto está en rutina de frenado, deteniendo la lógica de carril.
        """
        if self.estado_vehiculo == "PARADO":
            if tiempo_actual >= self.tiempo_fin_paro:
                # El tiempo de espera terminó, reanudamos marcha
                self.estado_vehiculo = "CONDUCIENDO"
                self.tiempo_ignorar_senal = tiempo_actual + self.get_parameter('tiempo_cooldown').value
                self.get_logger().info("Fin de la espera. Reanudando marcha y activando Cooldown temporal.")
                return False
            else:
                # Seguimos en espera, enviamos comando de freno a los motores
                self.publicar_comando_freno()
                return True
                
        return False

    def publicar_comando_freno(self):
        """Construye y envía un comando seguro para detener los motores."""
        cmd = MotorCommand()
        cmd.dir_dc = 0  # 0 suele ser alto/freno
        cmd.speed_dc = 0
        cmd.dir_servo = int(self.last_servo_value) # Mantenemos las llantas apuntando donde estaban
        cmd.turn_signals = 3 # Opcional: Encender luces de emergencia si el mensaje lo soporta
        self.pub_motor.publish(cmd)

    # ==========================================================
    # MÓDULO DE CARRIL (Lógica Principal)
    # ==========================================================
    def error_callback(self, msg):
        ahora_obj = self.get_clock().now()
        ahora_sec = ahora_obj.nanoseconds / 1e9
        
        # 1. EVALUAR FRENADO: Si la función devuelve True, abortamos el cálculo de carril este ciclo.
        if self.verificar_y_aplicar_freno(ahora_sec):
            self.last_time = ahora_obj # Reseteamos el timer PD para que no dé un salto brusco al reanudar
            return

        # --- A PARTIR DE AQUÍ SOLO SE EJECUTA SI EL AUTO ESTÁ "CONDUCIENDO" ---
        error_actual = msg.data
        
        # CÁLCULO DE TIEMPO (dt)
        dt = (ahora_obj - self.last_time).nanoseconds / 1e9
        self.last_time = ahora_obj
        if dt <= 0: dt = 0.033 

        # LEER PARÁMETROS DESDE RQT
        vel_base = self.get_parameter('velocidad_dc').value
        kp = self.get_parameter('kp_servo').value
        kd = self.get_parameter('kd_servo').value
        base_step = self.get_parameter('max_step_servo').value
        f_red = self.get_parameter('factor_reduccion_vel').value

        # CÁLCULO CONTROL PD
        P = error_actual * kp
        D = kd * (error_actual - self.error_previo) / dt
        self.error_previo = error_actual

        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # SUAVIZADO DE GIRO (SLEW RATE)
        max_step = base_step + int(abs(error_actual) * 20)
        diff = target_servo - self.last_servo_value
        
        if abs(diff) > max_step:
            nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
        else:
            nuevo_servo = target_servo
            
        self.last_servo_value = nuevo_servo

        # LÓGICA DE VELOCIDAD ADAPTATIVA
        reduccion = 1.0 - (abs(error_actual)**2 * f_red)
        speed_dc = int(vel_base * max(0.2, reduccion))

        # PUBLICACIÓN DEL COMANDO
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
        # Freno de emergencia al apagar el nodo con Ctrl+C
        stop = MotorCommand()
        stop.dir_dc = 0
        stop.speed_dc = 0
        stop.dir_servo = 1500
        node.pub_motor.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()