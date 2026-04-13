#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from motor_msgs.msg import MotorCommand
import numpy as np

# ==============================================================================
# MULTIPLEXOR: CONTROL DE CARRIL + FRENADO (STOP & CRUCES) + RQT START/STOP
# ==============================================================================

class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        # -------- PARÁMETROS DINÁMICOS PARA RQT --------
        
        # 0. Botón de Seguridad (Start / Stop)
        self.declare_parameter('habilitar_conduccion', False)

        # 1. Control de Velocidad y Dirección (PD)
        self.declare_parameter('velocidad_dc', 80)           
        self.declare_parameter('velocidad_minima', 60)       
        self.declare_parameter('factor_reduccion_vel', 20)   
        self.declare_parameter('kp_servo', 1500.0)            
        self.declare_parameter('kd_servo', 200.0)             
        self.declare_parameter('max_step_servo', 25)         

        # 2. Control de Frenado (Tiempos y distancias)
        self.declare_parameter('tiempo_detenido', 5.0)       # Segundos que el auto debe quedarse quieto
        self.declare_parameter('tiempo_ignorar', 4.0)        # Segundos de "cooldown" post-frenado
        
        # 3. Umbrales de los Sensores
        self.declare_parameter('distancia_paro_min', 0.15)   # Señal STOP: Distancia mínima
        self.declare_parameter('distancia_paro_max', 0.40)   # Señal STOP: Distancia máxima
        self.declare_parameter('linea_freno_cruce', 350.0)   # CRUCE: Coordenada Y (px) para frenar

        # Memorias para PD y Servo
        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        self.error_previo = 0.0
        self.last_time = self.get_clock().now()

        # Memorias de Sensores (Vision)
        self.distancia_senal = 999.0
        self.distancia_cruce = 999.0
        
        # Máquina de estados
        self.estado_vehiculo = "CONDUCIENDO" 
        self.tiempo_cambio_estado = self.get_clock().now()

        # -------- SUBSCRIPCIONES --------
        self.sub_error = self.create_subscription(
            Float32, '/steering_error', self.error_callback, 10)

        self.sub_distancia = self.create_subscription(
            Float32, '/vision/senal_stop/distancia_nube', self.distancia_callback, 10)
            
        self.sub_cruce = self.create_subscription(
            Float32, '/vision/cruce_peatonal/distancia', self.cruce_callback, 10)

        # -------- PUBLICADOR --------
        self.pub_motor = self.create_publisher(
            MotorCommand, '/motor_command', 10)

        self.get_logger().info("Nodo Multiplexor (PD + STOP + Cruces Peatonales) Iniciado")

    # --- CALLBACKS DE VISIÓN ---
    def distancia_callback(self, msg):
        self.distancia_senal = msg.data
        
    def cruce_callback(self, msg):
        self.distancia_cruce = msg.data

    # --- MÁQUINA DE ESTADOS DE FRENADO ---
    def evaluar_estado_frenado(self):
        ahora = self.get_clock().now()
        dt_estado = (ahora - self.tiempo_cambio_estado).nanoseconds / 1e9

        d_min = self.get_parameter('distancia_paro_min').value
        d_max = self.get_parameter('distancia_paro_max').value
        linea_cruce = self.get_parameter('linea_freno_cruce').value
        t_paro = self.get_parameter('tiempo_detenido').value
        t_ign = self.get_parameter('tiempo_ignorar').value

        # Evaluamos ambos sensores
        condicion_stop = (d_min <= self.distancia_senal <= d_max)
        condicion_cruce = (self.distancia_cruce >= linea_cruce) and (self.distancia_cruce != 999.0)

        if self.estado_vehiculo == "CONDUCIENDO":
            if condicion_stop or condicion_cruce:
                motivo = "CRUCE PEATONAL" if condicion_cruce else "SEÑAL STOP"
                self.estado_vehiculo = "DETENIDO"
                self.tiempo_cambio_estado = ahora
                self.get_logger().warn(f"¡{motivo} DETECTADO! Activando Freno Electromagnético ({t_paro}s).")
                return True
            return False

        elif self.estado_vehiculo == "DETENIDO":
            if dt_estado >= t_paro:
                self.estado_vehiculo = "IGNORANDO"
                self.tiempo_cambio_estado = ahora
                # Limpiamos las memorias de los sensores para no volver a frenar al instante
                self.distancia_senal = 999.0 
                self.distancia_cruce = 999.0
                self.get_logger().info("Tiempo de parada cumplido. Reanudando marcha...")
                return False
            return True

        elif self.estado_vehiculo == "IGNORANDO":
            if dt_estado >= t_ign:
                self.estado_vehiculo = "CONDUCIENDO"
                self.distancia_senal = 999.0
                self.distancia_cruce = 999.0
                self.get_logger().info("Periodo de gracia finalizado. Detección activa.")
            return False

    # --- BUCLE PRINCIPAL (CONTROL) ---
    def error_callback(self, msg):
        ahora = self.get_clock().now()
        
        # 0. BOTÓN START/STOP EN RQT
        modo_activo = self.get_parameter('habilitar_conduccion').value
        if not modo_activo:
            cmd_stop = MotorCommand(dir_dc=0, speed_dc=0, dir_servo=int(self.last_servo_value), turn_signals=0)
            self.pub_motor.publish(cmd_stop)
            self.last_time = ahora
            return

        # 1. TIEMPO (dt)
        dt = (ahora - self.last_time).nanoseconds / 1e9
        self.last_time = ahora
        if dt <= 0: dt = 0.033 

        error_actual = msg.data

        # 2. PARÁMETROS PD Y VELOCIDAD
        vel_base = self.get_parameter('velocidad_dc').value
        vel_min = self.get_parameter('velocidad_minima').value
        kp = self.get_parameter('kp_servo').value
        kd = self.get_parameter('kd_servo').value
        base_step = self.get_parameter('max_step_servo').value
        f_red = self.get_parameter('factor_reduccion_vel').value

        # 3. CONTROL PD (Dirección)
        P = error_actual * kp
        D = kd * (error_actual - self.error_previo) / dt
        self.error_previo = error_actual

        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # Slew Rate (Suavizado mecánico)
        max_step = base_step + int(abs(error_actual) * 20)
        diff = target_servo - self.last_servo_value
        if abs(diff) > max_step:
            nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
        else:
            nuevo_servo = target_servo
        self.last_servo_value = nuevo_servo

        # 4. VELOCIDAD ADAPTATIVA (Anti-Deadband)
        speed_sugerida = vel_base - (abs(error_actual) * f_red)
        speed_calculada = int(max(vel_min, min(speed_sugerida, vel_base)))

        # 5. MULTIPLEXOR: ¿Frenar o Avanzar?
        if self.evaluar_estado_frenado():
            speed_dc = 0 
            direccion_dc = 0 # Activa Freno Electromagnético
        else:
            speed_dc = speed_calculada
            direccion_dc = 1 # Marcha adelante

        # 6. COMANDO FINAL
        cmd = MotorCommand()
        cmd.dir_dc = direccion_dc
        cmd.speed_dc = speed_dc
        cmd.dir_servo = int(nuevo_servo)
        
        # Luces direccionales
        if error_actual > 0.3: cmd.turn_signals = 1 
        elif error_actual < -0.3: cmd.turn_signals = 2 
        else: cmd.turn_signals = 0

        self.pub_motor.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DriveCarrilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        stop = MotorCommand(dir_dc=0, speed_dc=0, dir_servo=1500, turn_signals=0)
        node.pub_motor.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()