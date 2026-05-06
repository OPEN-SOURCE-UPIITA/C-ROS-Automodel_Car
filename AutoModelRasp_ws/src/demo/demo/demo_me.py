#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Importamos las clases con la lógica de las maniobras
from demo.maniobras_seniales import ManiobrasSeniales
from demo.maniobras_rebase import ManiobrasRebase

# Importamos el mensaje oficial para los motores y luces
from motor_msgs.msg import MotorCommand

class DemoMaquinaEstados(Node):
    def __init__(self):
        super().__init__('demo_autonomo')
        self.get_logger().info("Iniciando Cerebro: Máquina de Estados Demo...")

        # ==========================================================
        # 1. LÓGICA EXTERNA (MÚSCULOS)
        # ==========================================================
        self.maniobras_seniales = ManiobrasSeniales(servo_centro=1500)
        self.maniobras_rebase = ManiobrasRebase(servo_centro=1500)

        # ==========================================================
        # 2. BANDERAS (LO QUE ESCUCHAN LAS OREJAS DEL NODO)
        # ==========================================================
        self.bandera_vision_senales = 0  # Viene de la cámara RGB (0 a 5)
        self.bandera_rebase_depth = 0    # Viene del radar 3D (0, 1, 6 o 7)
        self.bandera_valor_servo = 1500  # Hacia dónde apunta el camino libre
        
        # Memoria interna
        self.estado_actual = 0 
        self.tiempo_cambio_estado = self.get_clock().now()

        # ==========================================================
        # 3. SUSCRIPCIONES (CONECTANDO LAS OREJAS A LOS TÓPICOS)
        # ==========================================================
        self.sub_senales = self.create_subscription(
            Int32, '/vision/bandera_senales', self.cb_senales, 10)
            
        self.sub_rebase = self.create_subscription(
            Int32, '/vision/bandera_rebase', self.cb_rebase, 10)
            
        self.sub_direccion = self.create_subscription(
            Int32, '/vision/valor_servo', self.cb_direccion, 10)

        # ==========================================================
        # 4. PUBLICADOR (BOCA DEL NODO)
        # ==========================================================
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)
        
        # Bucle a 30Hz
        self.timer = self.create_timer(0.033, self.bucle_principal)

    # --- CALLBACKS (Solo guardan la info que va llegando) ---
    def cb_senales(self, msg): self.bandera_vision_senales = msg.data
    def cb_rebase(self, msg): self.bandera_rebase_depth = msg.data
    def cb_direccion(self, msg): self.bandera_valor_servo = msg.data


    # ==========================================================
    # 5. CEREBRO (BUCLE PRINCIPAL)
    # ==========================================================
    def bucle_principal(self):
        ahora = self.get_clock().now()
        dt_estado = (ahora - self.tiempo_cambio_estado).nanoseconds / 1e9

        # --- TOMA DE DECISIONES: PRIORIDAD ---
        # Por defecto, queremos hacerle caso a las señales de la calle
        nuevo_estado = self.bandera_vision_senales

        # PERO, si el radar 3D detecta un obstáculo (su bandera ya no es 0)...
        if self.bandera_rebase_depth != 0:
            # ... ignoramos la señal de la calle y forzamos el estado de rebase/emergencia
            nuevo_estado = self.bandera_rebase_depth

        # Si el estado cambió, reiniciamos el cronómetro interno
        if self.estado_actual != nuevo_estado:
            self.estado_actual = nuevo_estado
            self.tiempo_cambio_estado = ahora
            dt_estado = 0.0 # Reseteamos el dt para el nuevo estado

        # Variables seguras por defecto
        dir_dc, speed_dc, dir_servo, stop_l, turn_l = 1, 0, 1500, 0, 0

        # --- EJECUCIÓN DE LA MANIOBRA ---
        if self.estado_actual == 0:
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_seniales.logica_normal(self.bandera_valor_servo)
            
        elif self.estado_actual == 1:
            # Funciona tanto si la RGB ve un "Stop" como si el radar manda "Freno Emergencia"
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_seniales.logica_stop(dt_estado)
            
        elif self.estado_actual == 2:
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_seniales.logica_zona_escolar(self.bandera_valor_servo)
            
        elif self.estado_actual == 3:
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_seniales.logica_giro(dt_estado, direccion="izquierda")
            
        elif self.estado_actual == 4:
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_seniales.logica_derrapante(dt_estado)
            
        elif self.estado_actual == 5:
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_seniales.logica_100kmh(self.bandera_valor_servo)

        elif self.estado_actual == 6:
            # Rebase por la derecha (pedido por el radar)
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_rebase.logica_rebase_derecha(self.bandera_valor_servo)
            
        elif self.estado_actual == 7:
            # Rebase por la izquierda (pedido por el radar)
            dir_dc, speed_dc, dir_servo, stop_l, turn_l = self.maniobras_rebase.logica_rebase_izquierda(self.bandera_valor_servo)

        # --- EMPAQUETAR Y PUBLICAR ---
        cmd = MotorCommand()
        cmd.dir_dc = int(dir_dc)
        cmd.speed_dc = int(speed_dc)
        cmd.dir_servo = int(dir_servo)
        cmd.stop_lights = int(stop_l)
        cmd.turn_signals = int(turn_l)

        self.pub_motor.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    nodo = DemoMaquinaEstados()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # Freno de emergencia al apagar el nodo
        freno = MotorCommand(dir_dc=0, speed_dc=0, dir_servo=1500, stop_lights=1, turn_signals=0)
        nodo.pub_motor.publish(freno)
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()