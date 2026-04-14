#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
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
        self.distancia_segura_atras = 0.8  # Metros para considerar que ya lo pasamos
        self.obstaculo_atras = False

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
        self.sub_radar = self.create_subscription(
            Bool, '/radar/alerta_rebase', self.radar_callback, 10)
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)

        self.get_logger().info("Nodo Drive Carril v3 (Timed State Machine) listo.")

    def lidar_callback(self, msg):
        # Si no estamos en medio de un rebase, no nos importa mucho el punto ciego
        if self.estado_maniobra != "REBASANDO":
            return

        # Definimos el sector de búsqueda según el carril al que fuimos
        # Si estamos en el IZQUIERDO, buscamos al auto que dejamos en el DERECHO (atrás-derecha)
        if self.carril_objetivo == "IZQUIERDO":
            # Grados 110 a 150 (ajustar según posición física del LiDAR)
            sector_atras = msg.ranges[110:150]
        else:
            # Si estamos en el DERECHO, buscamos atrás-izquierda (210 a 250)
            sector_atras = msg.ranges[210:250]

        distancia_atras = min(sector_atras)

        # Si la distancia es mayor a nuestra distancia de seguridad, el camino está libre
        if distancia_atras > self.distancia_segura_atras:
            self.get_logger().info(f"Punto ciego despejado ({distancia_atras:.2f}m). Regresando al carril...")
            
            # Disparamos el regreso al carril original
            self.carril_objetivo = "DERECHO" if self.carril_objetivo == "IZQUIERDO" else "IZQUIERDO"
            
            # Actualizamos RQT
            p = rclpy.parameter.Parameter('carril_derecho_activo', rclpy.Parameter.Type.BOOL, (self.carril_objetivo == "DERECHO"))
            self.set_parameters([p])
            
            # Reiniciamos la máquina de estados para el giro de vuelta
            self.estado_maniobra = "IMPULSO"
            self.inicio_sub_maniobra = self.get_clock().now()


    def radar_callback(self, msg):
        # Solo reaccionamos si el radar detecta un obstáculo (True)
        # Y si NO estamos ya haciendo una maniobra (para no interrumpir el proceso)
        if msg.data and self.estado_maniobra == "SEGUIMIENTO":
            self.get_logger().error("¡OBSTÁCULO DETECTADO! Iniciando rebase automático...")
            
            # Cambiamos el carril objetivo al opuesto
            self.carril_objetivo = "IZQUIERDO" if self.carril_objetivo == "DERECHO" else "DERECHO"
            
            # Actualizamos el parámetro en RQT para que veas visualmente que cambió
            # Nota: Esto es opcional, pero ayuda mucho a debugear
            param_carril = rclpy.parameter.Parameter(
                'carril_derecho_activo', 
                rclpy.Parameter.Type.BOOL, 
                (self.carril_objetivo == "DERECHO")
            )
            self.set_parameters([param_carril])

            # Disparamos la máquina de estados
            self.estado_maniobra = "IMPULSO"
            self.inicio_sub_maniobra = self.get_clock().now()

    def carril_callback(self, msg):
        self.carril_detectado = msg.data

    def distancia_callback(self, msg):
        self.distancia_senal = msg.data

    def gestionar_objetivo_carril(self, error_vision):
        """Máquina de estados: Salida ciega -> Rebase con visión -> Regreso ciego."""
        ahora = self.get_clock().now()
        
        # 1. Monitoreo del Objetivo (Cambia por RQT, Radar o LiDAR)
        carril_switch = self.get_parameter('carril_derecho_activo').value
        nuevo_objetivo = "DERECHO" if carril_switch else "IZQUIERDO"

        # Si el carril objetivo cambia (ya sea por el radar o manual), iniciamos la maniobra
        if nuevo_objetivo != self.carril_objetivo:
            self.carril_objetivo = nuevo_objetivo
            self.estado_maniobra = "IMPULSO"
            self.inicio_sub_maniobra = ahora
            self.get_logger().warn(f"Maniobra Iniciada hacia: {self.carril_objetivo}")

        # 2. Lógica de la Máquina de Estados
        dt = (ahora - self.inicio_sub_maniobra).nanoseconds / 1e9
        fuerza = self.get_parameter('fuerza_rebase_pct').value / 100.0

        if self.estado_maniobra == "SEGUIMIENTO":
            return error_vision

        elif self.estado_maniobra == "IMPULSO":
            if dt < self.t_impulso:
                return fuerza if self.carril_objetivo == "DERECHO" else -fuerza
            else:
                self.estado_maniobra, self.inicio_sub_maniobra = "REINTEGRACION", ahora
                return 0.0

        elif self.estado_maniobra == "REINTEGRACION":
            if dt < self.t_reintegracion:
                return -fuerza if self.carril_objetivo == "DERECHO" else fuerza
            else:
                # Después de enderezarse, pasamos a RECUPERACION para limpiar la visión
                self.estado_maniobra, self.inicio_sub_maniobra = "RECUPERACION", ahora
                return 0.0

        elif self.estado_maniobra == "RECUPERACION":
            if dt < self.t_recuperacion:
                return 0.0
            else:
                # Una vez que la visión es estable, nos quedamos en REBASANDO
                # En este estado seguimos las líneas del NUEVO carril
                self.get_logger().info(f"Estable en carril {self.carril_objetivo}. Esperando despeje del LiDAR.")
                self.estado_maniobra = "REBASANDO"
                return error_vision

        elif self.estado_maniobra == "REBASANDO":
            # El coche sigue manejando normal. 
            # Solo saldrá de aquí cuando el lidar_callback cambie el parámetro 'carril_derecho_activo'
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
