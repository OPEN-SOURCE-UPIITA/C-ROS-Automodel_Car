#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String  # Importamos String para leer el carril
from motor_msgs.msg import MotorCommand
import numpy as np

class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        # -------- PARÁMETROS DINÁMICOS PARA RQT --------
        self.declare_parameter('habilitar_conduccion', False)
        
        # NUEVO: Parámetro tipo Checkbox para disparar el rebase/cambio
        self.declare_parameter('cambiar_carril', False)
        self.last_button_state = False # Para detectar el "click" (flanco de subida)

        self.declare_parameter('velocidad_dc', 80)
        self.declare_parameter('velocidad_minima', 60)
        self.declare_parameter('factor_reduccion_vel', 20)
        self.declare_parameter('kp_servo', 198.0)
        self.declare_parameter('kd_servo', 50.0)
        self.declare_parameter('max_step_servo', 25)

        # ... (resto de parámetros de señales igual) ...
        self.declare_parameter('distancia_paro_min', 0.15)
        self.declare_parameter('distancia_paro_max', 0.40)
        self.declare_parameter('tiempo_detenido', 5.0)
        self.declare_parameter('tiempo_ignorar', 4.0)

        # Variables de control
        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        self.error_previo = 0.0
        self.last_time = self.get_clock().now()
        
        # NUEVO: Gestión de carriles
        self.carril_detectado = "DESCONOCIDO"
        self.carril_objetivo = "DERECHO" # Por defecto empezamos en el derecho

        # Máquina de estados para Frenado
        self.distancia_senal = 999.0
        self.estado_vehiculo = "CONDUCIENDO"
        self.tiempo_cambio_estado = self.get_clock().now()

        # -------- SUBSCRIPCIONES Y PUBLICACIONES --------
        self.sub_error = self.create_subscription(
            Float32, '/steering_error', self.error_callback, 10)

        # NUEVO: Suscripción al carril que ve la visión
        self.sub_carril = self.create_subscription(
            String, '/vision/carril_actual', self.carril_callback, 10)

        self.sub_distancia = self.create_subscription(
            Float32, '/vision/senal_stop/distancia_nube', self.distancia_callback, 10)

        self.pub_motor = self.create_publisher(
            MotorCommand, '/motor_command', 10)

        self.get_logger().info("Nodo Drive Carril con Selector de Carril listo.")

    def carril_callback(self, msg):
        self.carril_detectado = msg.data

    def distancia_callback(self, msg):
        self.distancia_senal = msg.data

    def evaluar_estado_frenado(self):
        # ... (Se mantiene igual que tu código original) ...
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
                return True
            return False
        elif self.estado_vehiculo == "DETENIDO":
            if dt_estado >= t_paro:
                self.estado_vehiculo = "IGNORANDO"
                self.tiempo_cambio_estado = ahora
                return False
            return True
        elif self.estado_vehiculo == "IGNORANDO":
            if dt_estado >= t_ign:
                self.estado_vehiculo = "CONDUCIENDO"
            return False

    def error_callback(self, msg):
        ahora = self.get_clock().now()
        modo_activo = self.get_parameter('habilitar_conduccion').value
        
        if not modo_activo:
            cmd_stop = MotorCommand()
            cmd_stop.dir_dc = 0
            cmd_stop.speed_dc = 0
            cmd_stop.dir_servo = int(self.last_servo_value)
            self.pub_motor.publish(cmd_stop)
            self.last_time = ahora
            return

        # --- LÓGICA DE CAMBIO DE CARRIL (BANDERA RQT) ---
        boton_cambio = self.get_parameter('cambiar_carril').value
        
        # Detectamos cuando el usuario ACTIVA el checkbox (False -> True)
        if boton_cambio and not self.last_button_state:
            if self.carril_objetivo == "DERECHO":
                self.carril_objetivo = "IZQUIERDO"
            else:
                self.carril_objetivo = "DERECHO"
            self.get_logger().warn(f"¡CAMBIO DE CARRIL SOLICITADO! Nuevo objetivo: {self.carril_objetivo}")
        
        self.last_button_state = boton_cambio

        # --- PROCESAMIENTO DEL ERROR ---
        # Si estamos en el carril que queremos, el error es el que manda la cámara.
        # Si NO estamos en el carril objetivo, el error debe forzar el rebase.
        error_actual = msg.data

        # OFFSET TEMPORAL PARA REBASE (Si detectamos que estamos en el carril equivocado)
        # Esto es una ayuda por si el error de visión tarda en actualizarse al cambiar de carril
        if self.carril_detectado != self.carril_objetivo and self.carril_detectado != "DESCONOCIDO":
            # Si quiero ir a la izquierda y estoy en la derecha, mando un error positivo fuerte
            if self.carril_objetivo == "IZQUIERDO":
                error_actual -= 0.4 
            # Si quiero ir a la derecha y estoy en la izquierda, mando un error negativo fuerte
            else:
                error_actual += 0.4
        
        error_actual = np.clip(error_actual, -1.0, 1.0)

        # --- CONTROLADOR PD (Igual que el original) ---
        dt = (ahora - self.last_time).nanoseconds / 1e9
        self.last_time = ahora
        if dt <= 0: dt = 0.033 

        vel_base = self.get_parameter('velocidad_dc').value
        vel_min = self.get_parameter('velocidad_minima').value
        kp = self.get_parameter('kp_servo').value
        kd = self.get_parameter('kd_servo').value
        base_step = self.get_parameter('max_step_servo').value
        f_red = self.get_parameter('factor_reduccion_vel').value

        P = error_actual * kp
        D = kd * (error_actual - self.error_previo) / dt
        self.error_previo = error_actual

        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        max_step = base_step + int(abs(error_actual) * 20)
        diff = target_servo - self.last_servo_value
        nuevo_servo = self.last_servo_value + np.sign(diff) * max_step if abs(diff) > max_step else target_servo
        self.last_servo_value = nuevo_servo

        # Velocidad y Publicación
        speed_sugerida = vel_base - (abs(error_actual) * f_red)
        speed_calculada = int(max(vel_min, min(speed_sugerida, vel_base)))
        speed_dc = 0 if self.evaluar_estado_frenado() else speed_calculada

        cmd = MotorCommand()
        cmd.dir_dc = 1
        cmd.speed_dc = speed_dc
        cmd.dir_servo = int(nuevo_servo)
        
        # Luces (Usamos el objetivo para indicar a dónde queremos ir)
        if self.carril_objetivo == "IZQUIERDO": cmd.turn_signals = 1
        elif self.carril_objetivo == "DERECHO": cmd.turn_signals = 2
        else: cmd.turn_signals = 0

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
