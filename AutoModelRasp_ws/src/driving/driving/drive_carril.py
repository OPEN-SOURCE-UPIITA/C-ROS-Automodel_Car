#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from motor_msgs.msg import MotorCommand
import numpy as np

class DriveCarrilNode(Node):
    def __init__(self):
        super().__init__('drive_carril')

        # -------- PARÁMETROS RQT --------
        self.declare_parameter('habilitar_conduccion', False)
        self.declare_parameter('carril_derecho_activo', True) 
        self.declare_parameter('fuerza_rebase_pct', 60) # Ajustable en RQT (0-100)

        # Tiempos de maniobra (Puedes declararlos como parámetros si quieres tunear sin reiniciar)
        self.t_impulso = 1.2        # Tiempo de volantazo inicial
        self.t_reintegracion = 1.0  # Tiempo de contra-volante para enderezar
        self.t_recuperacion = 1.0    # Tiempo de espera para que la visión se estabilice

        self.declare_parameter('velocidad_dc', 80)
        self.declare_parameter('velocidad_minima', 60)
        self.declare_parameter('factor_reduccion_vel', 20)
        self.declare_parameter('kp_servo', 198.0)
        self.declare_parameter('kd_servo', 50.0)
        self.declare_parameter('max_step_servo', 25)

        # Parámetros de señales (STOP)
        self.declare_parameter('distancia_paro_min', 0.15)
        self.declare_parameter('distancia_paro_max', 0.40)
        self.declare_parameter('tiempo_detenido', 5.0)
        self.declare_parameter('tiempo_ignorar', 4.0)

        # Variables de Control y Estado
        self.servo_centro = 1500
        self.last_servo_value = self.servo_centro
        self.error_previo = 0.0
        self.last_time = self.get_clock().now()
        
        self.carril_detectado = "DESCONOCIDO"
        self.carril_objetivo = "DERECHO" if self.get_parameter('carril_derecho_activo').value else "IZQUIERDO"
        
        # MÁQUINA DE ESTADOS DE REBASE: "SEGUIMIENTO", "IMPULSO", "REINTEGRACION", "RECUPERACION"
        self.estado_maniobra = "SEGUIMIENTO"
        self.inicio_sub_maniobra = self.get_clock().now()

        # Máquina de estados para Frenado (Señales)
        self.distancia_senal = 999.0
        self.estado_vehiculo = "CONDUCIENDO"
        self.tiempo_cambio_estado = self.get_clock().now()

        # -------- SUBSCRIPCIONES Y PUBLICACIONES --------
        self.sub_error = self.create_subscription(Float32, '/steering_error', self.error_callback, 10)
        self.sub_carril = self.create_subscription(String, '/vision/carril_actual', self.carril_callback, 10)
        self.sub_distancia = self.create_subscription(Float32, '/vision/senal_stop/distancia_nube', self.distancia_callback, 10)
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)

        self.get_logger().info("Nodo Drive Carril v3 (Timed State Machine) listo.")

    def carril_callback(self, msg):
        self.carril_detectado = msg.data

    def distancia_callback(self, msg):
        self.distancia_senal = msg.data

    def gestionar_objetivo_carril(self, error_vision):
        """Implementa la máquina de estados que ignora la visión durante el rebase."""
        ahora = self.get_clock().now()
        
        # 1. Monitoreo del Checkbox de RQT
        carril_switch = self.get_parameter('carril_derecho_activo').value
        nuevo_objetivo = "DERECHO" if carril_switch else "IZQUIERDO"

        # Si el usuario cambia el switch, disparamos la máquina de estados
        if nuevo_objetivo != self.carril_objetivo:
            self.carril_objetivo = nuevo_objetivo
            self.estado_maniobra = "IMPULSO"
            self.inicio_sub_maniobra = ahora
            self.get_logger().warn(f"Iniciando Maniobra Ciega hacia: {self.carril_objetivo}")

        # 2. Lógica de Tiempos de la Máquina de Estados
        dt = (ahora - self.inicio_sub_maniobra).nanoseconds / 1e9
        fuerza = self.get_parameter('fuerza_rebase_pct').value / 100.0

        if self.estado_maniobra == "SEGUIMIENTO":
            return error_vision

        elif self.estado_maniobra == "IMPULSO":
            if dt < self.t_impulso:
                # Gira fuerte ignorando la visión
                return fuerza if self.carril_objetivo == "DERECHO" else -fuerza
            else:
                self.estado_maniobra = "REINTEGRACION"
                self.inicio_sub_maniobra = ahora
                return 0.0

        elif self.estado_maniobra == "REINTEGRACION":
            if dt < self.t_reintegracion:
                # Contra-giro para enderezar el coche antes de ver líneas
                return -fuerza if self.carril_objetivo == "DERECHO" else fuerza
            else:
                self.estado_maniobra = "RECUPERACION"
                self.inicio_sub_maniobra = ahora
                return 0.0

        elif self.estado_maniobra == "RECUPERACION":
            # Espera a que la cámara deje de ver basura tras el movimiento brusco
            if dt < self.t_recuperacion:
                return 0.0
            else:
                self.get_logger().info("Regresando control a la Visión.")
                self.estado_maniobra = "SEGUIMIENTO"
                return error_vision

        return error_vision

    def evaluar_estado_frenado(self):
        ahora = self.get_clock().now()
        dt_estado = (ahora - self.tiempo_cambio_estado).nanoseconds / 1e9
        d_min = self.get_parameter('distancia_paro_min').value
        d_max = self.get_parameter('distancia_paro_max').value
        t_paro = self.get_parameter('tiempo_detenido').value
        t_ign = self.get_parameter('tiempo_ignorar').value

        if self.estado_vehiculo == "CONDUCIENDO":
            if d_min <= self.distancia_senal <= d_max:
                self.estado_vehiculo, self.tiempo_cambio_estado = "DETENIDO", ahora
                return True
        elif self.estado_vehiculo == "DETENIDO":
            if dt_estado >= t_paro:
                self.estado_vehiculo, self.tiempo_cambio_estado = "IGNORANDO", ahora
                return False
            return True
        elif self.estado_vehiculo == "IGNORANDO":
            if dt_estado >= t_ign: self.estado_vehiculo = "CONDUCIENDO"
        return False

    def error_callback(self, msg):
        ahora = self.get_clock().now()
        if not self.get_parameter('habilitar_conduccion').value:
            self.parar_motores()
            self.last_time = ahora
            return

        # 1. Obtener error (Visión o Maniobra Ciega)
        error_final = self.gestionar_objetivo_carril(msg.data)
        error_final = np.clip(error_final, -1.0, 1.0)

        # 2. PD
        dt = (ahora - self.last_time).nanoseconds / 1e9
        self.last_time = ahora
        if dt <= 0: dt = 0.033

        kp, kd = self.get_parameter('kp_servo').value, self.get_parameter('kd_servo').value
        P = error_final * kp
        D = kd * (error_final - self.error_previo) / dt
        self.error_previo = error_final

        # 3. Dirección
        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # Suavizado de dirección
        max_step = self.get_parameter('max_step_servo').value + int(abs(error_final) * 20)
        diff = target_servo - self.last_servo_value
        nuevo_servo = self.last_servo_value + np.sign(diff) * max_step if abs(diff) > max_step else target_servo
        self.last_servo_value = nuevo_servo

        # 4. Velocidad
        vel_base = self.get_parameter('velocidad_dc').value
        f_red = self.get_parameter('factor_reduccion_vel').value
        speed_sugerida = vel_base - (abs(error_final) * f_red)
        speed_calculada = int(max(self.get_parameter('velocidad_minima').value, min(speed_sugerida, vel_base)))
        
        # Publicar
        self.publicar_comando(speed_calculada, nuevo_servo)

    def publicar_comando(self, speed, servo):
        cmd = MotorCommand()
        cmd.dir_dc = 1
        cmd.speed_dc = 0 if self.evaluar_estado_frenado() else speed
        cmd.dir_servo = int(servo)
        
        if self.carril_objetivo == "IZQUIERDO": cmd.turn_signals = 1
        elif self.carril_objetivo == "DERECHO": cmd.turn_signals = 2
        else: cmd.turn_signals = 0
        
        self.pub_motor.publish(cmd)

    def parar_motores(self):
        cmd = MotorCommand()
        cmd.dir_dc, cmd.speed_dc = 0, 0
        cmd.dir_servo = int(self.last_servo_value)
        self.pub_motor.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DriveCarrilNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.parar_motores()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
