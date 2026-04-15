#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Float32MultiArray
from rcl_interfaces.msg import SetParametersResult
from motor_msgs.msg import MotorCommand

# Importamos las librerías matemáticas (asegúrate de poner el nombre de tu paquete)
from driving.controlador_pd import ControladorAutomodel
from driving.logica_maniobras import ManiobrasAutonomo

class AutonomoNode(Node):
    def __init__(self):
        super().__init__('autonomo')
        self.get_logger().info("Iniciando Cerebro Autónomo...")

        # --- LIBRERÍAS EXTERNAS ---
        self.pd = ControladorAutomodel(servo_centro=1500)
        self.maniobras = ManiobrasAutonomo(controlador_pd=self.pd)

        # --- PARÁMETROS DINÁMICOS ---
        self.declare_parameter('habilitar_conduccion', False)
        self.declare_parameter('velocidad_base', 80)
        self.declare_parameter('velocidad_minima', 60)
        self.declare_parameter('factor_reduccion', 20)
        self.declare_parameter('kp', 1500.0)
        self.declare_parameter('kd', 200.0)
        self.declare_parameter('max_step', 25)
        
        self.declare_parameter('tiempo_stop', 3.0)
        self.declare_parameter('tiempo_cruce', 4.0)
        self.declare_parameter('tiempo_gracia_senales', 3.0) # Segundos sin leer señales tras maniobra

        # --PARAMETROS DE REBASE--
        self.declare_parameter('rebase.volantazo_intensidad', 600)
        self.declare_parameter('rebase.tiempo_volantazo', 0.35)
        self.declare_parameter('rebase.contravolantazo_intensidad', 250)
        self.declare_parameter('rebase.tiempo_contravolantazo', 0.25)
        self.declare_parameter('rebase.velocidad_rebase', 85)
        self.declare_parameter('rebase.tiempo_minimo_rebase', 1.5)
        self.declare_parameter('rebase.tiempo_maximo_rebase', 4.0)
        self.declare_parameter('rebase.distancia_segura', 0.50)
        self.declare_parameter('rebase.distancia_peligro', 0.35)
        self.declare_parameter('rebase.regreso_intensidad', 400)
        self.declare_parameter('rebase.tiempo_regreso', 0.40)

        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- ESTADO DEL SISTEMA ---
        # 0: CARRIL (PD), 1: STOP, 2: CRUCE, 3: REBASE, 4: GRACIA (Ignorar señales temporalmente)
        self.estado_actual = 0 
        self.tiempo_cambio_estado = self.get_clock().now()

        # --- VARIABLES DE SENSORES ---
        self.error_carril = 0.0
        self.carril_actual = "DESCONOCIDO"
        self.distancia_stop = 999.0
        self.distancia_cruce = 999.0
        self.obstaculo_rebase = False

        # --- SUSCRIPCIONES ---
        self.create_subscription(Float32, '/steering_error', self.error_cb, 10)
        self.create_subscription(String, '/vision/carril_actual', self.carril_actual_cb, 10)
        self.create_subscription(Float32, '/vision/distancia_real', self.stop_cb, 10)
        self.create_subscription(Float32, '/vision/cruce_peatonal/distancia', self.cruce_cb, 10)
        self.create_subscription(Bool, '/radar/alerta_rebase', self.rebase_cb, 10)
        self.create_subscription(Float32MultiArray, '/radar/distancias_franjas', self.lidar_distancias_cb, 10)

        # --- PUBLICADOR ---
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)

        # --- BUCLE PRINCIPAL (30 Hz) ---
        self.timer = self.create_timer(0.033, self.bucle_principal)
        self.last_time = self.get_clock().now()

    # --- CALLBACKS (Solo guardan datos) ---

    def error_cb(self, msg): self.error_carril = msg.data
    def carril_actual_cb(self, msg): self.carril_actual = msg.data
    def stop_cb(self, msg): self.distancia_stop = msg.data
    def cruce_cb(self, msg): self.distancia_cruce = msg.data
    def rebase_cb(self, msg): self.obstaculo_rebase = msg.data
    def lidar_distancias_cb(self, msg): self.maniobras.actualizar_distancias_lidar(msg.data)

    def parameters_callback(self, params):
        # Actualizar parámetros del rebase
        rebase_params = {}
        for param in params:
            if param.name.startswith('rebase.'):
                key = param.name.split('.')[1]
                rebase_params[key] = param.value
    
        if rebase_params:
            self.maniobras.actualizar_parametros_rebase(rebase_params)
    
        return SetParametersResult(successful=True)

    # --- CEREBRO ---
    def bucle_principal(self):
        ahora = self.get_clock().now()
        dt = (ahora - self.last_time).nanoseconds / 1e9
        self.last_time = ahora
        if dt <= 0: dt = 0.033

        dt_estado = (ahora - self.tiempo_cambio_estado).nanoseconds / 1e9

        # 0. BOTÓN DE EMERGENCIA RQT
        if not self.get_parameter('habilitar_conduccion').value:
            cmd = MotorCommand(dir_dc=0, speed_dc=0, dir_servo=self.pd.last_servo_value, turn_signals=0)
            self.pub_motor.publish(cmd)
            self.tiempo_cambio_estado = ahora # Mantenemos el timer reiniciado
            return

        vel_cmd = 0
        servo_cmd = 1500
        dir_dc = 1 # 1 = Adelante, 0 = Freno

        # ==========================================
        # MÁQUINA DE ESTADOS
        # ==========================================

        # ESTADO 0: CONDUCCIÓN NORMAL
        if self.estado_actual == 0:
            # 1. Evaluar si cambiamos de estado (Prioridades: Obstáculo > Stop > Cruce)
            if self.obstaculo_rebase:
                self.estado_actual = 3
                self.tiempo_cambio_estado = ahora
                self.get_logger().warn("¡Iniciando REBASE!")
            elif self.distancia_stop < 0.40: # Umbral de frenado Stop
                self.estado_actual = 1
                self.tiempo_cambio_estado = ahora
                self.get_logger().warn("¡STOP Detectado! Frenando.")
            elif self.distancia_cruce < 350.0: # Umbral de frenado Cruce
                self.estado_actual = 2
                self.tiempo_cambio_estado = ahora
                self.get_logger().warn("¡CRUCE Detectado! Frenando.")
            else:
                # 2. Si no hay cambios, ejecutar PD
                kp = self.get_parameter('kp').value
                kd = self.get_parameter('kd').value
                vel_base = self.get_parameter('velocidad_base').value
                vel_min = self.get_parameter('velocidad_minima').value
                f_red = self.get_parameter('factor_reduccion').value
                max_st = self.get_parameter('max_step').value

                servo_cmd = self.pd.calcular_direccion_pd(self.error_carril, dt, kp, kd, max_st)
                vel_cmd = self.pd.calcular_velocidad_adaptativa(self.error_carril, vel_base, vel_min, f_red)

        # ESTADO 1: LÓGICA DE STOP
        elif self.estado_actual == 1:
            t_stop = self.get_parameter('tiempo_stop').value
            vel_cmd, servo_cmd, terminado = self.maniobras.ejecutar_logica_stop(dt_estado, t_stop)
            dir_dc = 0 # Freno
            
            if terminado:
                self.estado_actual = 4 # Pasar a periodo de gracia
                self.tiempo_cambio_estado = ahora

        # ESTADO 2: LÓGICA DE CRUCE
        elif self.estado_actual == 2:
            t_cruce = self.get_parameter('tiempo_cruce').value
            vel_cmd, servo_cmd, terminado = self.maniobras.ejecutar_logica_cruce(dt_estado, t_cruce)
            dir_dc = 0
            
            if terminado:
                self.estado_actual = 4
                self.tiempo_cambio_estado = ahora

        # ESTADO 3: LÓGICA DE REBASE
        elif self.estado_actual == 3:
            vel_cmd, servo_cmd, terminado = self.maniobras.ejecutar_logica_rebase(dt_estado, self.carril_actual, self.error_carril)
            dir_dc = 1
            
            if terminado:
                self.estado_actual = 4
                self.tiempo_cambio_estado = ahora
                self.get_logger().info("Rebase terminado. Volviendo a carril.")

        # ESTADO 4: PERIODO DE GRACIA (Ignorar señales para no trabarse)
        elif self.estado_actual == 4:
            t_gracia = self.get_parameter('tiempo_gracia_senales').value
            
            # Conducimos normal con el PD
            servo_cmd = self.pd.calcular_direccion_pd(self.error_carril, dt, self.get_parameter('kp').value, self.get_parameter('kd').value, self.get_parameter('max_step').value)
            vel_cmd = self.pd.calcular_velocidad_adaptativa(self.error_carril, self.get_parameter('velocidad_base').value, self.get_parameter('velocidad_minima').value, self.get_parameter('factor_reduccion').value)
            
            if dt_estado >= t_gracia:
                self.estado_actual = 0 # Volver a buscar señales
                # Limpiar memorias para no frenar instantáneamente
                self.distancia_stop = 999.0 
                self.distancia_cruce = 999.0
                self.obstaculo_rebase = False

        # ==========================================
        # PUBLICAR A MOTORES
        # ==========================================
        cmd = MotorCommand()
        cmd.dir_dc = dir_dc
        cmd.speed_dc = vel_cmd
        cmd.dir_servo = int(servo_cmd)
        
        # Luces dinámicas
        if self.estado_actual == 3 and servo_cmd < 1400: cmd.turn_signals = 1 # Izquierda
        elif self.estado_actual == 3 and servo_cmd > 1600: cmd.turn_signals = 2 # Derecha
        elif self.error_carril > 0.3: cmd.turn_signals = 1 
        elif self.error_carril < -0.3: cmd.turn_signals = 2 
        else: cmd.turn_signals = 0

        self.pub_motor.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomoNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        stop = MotorCommand(dir_dc=0, speed_dc=0, dir_servo=1500, turn_signals=0)
        node.pub_motor.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
