import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
import math

class AutoTunerPI(Node):
    def __init__(self):
        super().__init__('auto_tuner_pi')
        
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.encoder_cb, 10)
        
        # --- CONFIGURACIÓN FÍSICA ---
        self.ticks_por_metro = 2060.0 
        self.distancia_1d = 0.0 
        self.comando_dir = 1 
        
        # --- ESTRATEGIA DE DOS FASES ---
        self.fase_actual = 1
        
        # FASE 1: Exploración amplia de Kp y Ki
        # Probaremos Kp desde 20 a 100 y Ki de 0 a 15
        self.lista_pruebas = [(float(kp), float(ki)) for kp in [20, 40, 60, 80, 100] for ki in [0, 5, 10, 15]]
        self.indice_prueba = 0
        self.resultados = [] 
        
        self.target_dist = 1.0  # Meta: 1 metro
        self.error_integral = 0.0
        self.error_acumulado_prueba = 0.0 
        
        self.dt = 0.1
        self.estado = 'TESTING'
        self.ciclos_actuales = 0
        self.ciclos_pausa = 0

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("AUTO-TUNER V2 (Sincronizado con nuevo Adaptador) ")
        self.get_logger().info("Iniciando Fase 1: Grano Grueso...")

    def encoder_cb(self, msg):
        # El nuevo adaptador envía deltas, los sumamos considerando la dirección actual
        signo = 1 if self.comando_dir == 1 else -1
        
        # Promedio de ticks de ambas ruedas
        ticks_promedio = (abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0
        distancia_paso = (ticks_promedio / self.ticks_por_metro) * signo
        
        # Filtro para evitar saltos locos por ruido
        if abs(distancia_paso) < 0.15:
            self.distancia_1d += distancia_paso

    def control_loop(self):
        cmd = MotorCommand()
        cmd.dir_servo = 1500 
        cmd.turn_signals = 3 # Luces de advertencia
        
        if self.estado == 'TERMINADO':
            cmd.dir_dc, cmd.speed_dc = 0, 0
            cmd.stop_lights = 1
            self.pub_cmd.publish(cmd)
            return

        if self.estado == 'PAUSA':
            cmd.dir_dc, cmd.speed_dc = 0, 0
            self.ciclos_pausa += 1
            if self.ciclos_pausa >= 25: # 2.5 seg de pausa para frenado total
                self.estado = 'TESTING'
                self.ciclos_pausa = 0
                self.ciclos_actuales = 0
                self.error_integral = 0.0
                self.error_acumulado_prueba = 0.0
                # Alternar meta para ida y vuelta
                self.target_dist = 0.0 if self.target_dist == 1.0 else 1.0
            self.pub_cmd.publish(cmd)
            return

        if self.estado == 'TESTING':
            kp_actual, ki_actual = self.lista_pruebas[self.indice_prueba]
            
            error = self.target_dist - self.distancia_1d
            self.error_acumulado_prueba += abs(error)
            
            # Algoritmo PI
            self.error_integral += error * self.dt
            self.error_integral = max(-1.0, min(1.0, self.error_integral)) # Anti-windup
            
            velocidad_calculada = (kp_actual * error) + (ki_actual * self.error_integral)
            
            # Aplicamos PWM (Máximo 80 para no ser tan agresivos en el simulador)
            velocidad_pwm = int(min(80.0, abs(velocidad_calculada)))
            
            if velocidad_calculada > 0:
                cmd.dir_dc = 1 
            else:
                cmd.dir_dc = 2 
                
            # Banda muerta estricta (1 milímetro)
            if abs(error) <= 0.001:
                cmd.dir_dc, velocidad_pwm = 0, 0
                
            cmd.speed_dc = velocidad_pwm
            self.comando_dir = cmd.dir_dc
            self.ciclos_actuales += 1
            
            if self.ciclos_actuales % 10 == 0:
                self.get_logger().info(f"[F{self.fase_actual} P{self.indice_prueba+1}] Kp:{kp_actual} Ki:{ki_actual} | Err:{error:.4f}m")

            # Cada prueba dura 5 segundos
            if self.ciclos_actuales >= 50:
                err_f = abs(error)
                self.resultados.append((kp_actual, ki_actual, err_f))
                
                # PARADA DE ÉXITO: Si ya estamos bajo 5mm en fase 2
                if self.fase_actual == 2 and err_f <= 0.005:
                    self.estado = 'TERMINADO'
                    self.imprimir_ganador(kp_actual, ki_actual, err_f)
                    return

                self.indice_prueba += 1
                
                # Gestión de Fases
                if self.indice_prueba >= len(self.lista_pruebas):
                    if self.fase_actual == 1:
                        self.cambiar_a_fase_2()
                    else:
                        # Si termina todo sin el ideal, agarra el mejor
                        mejor = min(self.resultados, key=lambda x: x[2])
                        self.estado = 'TERMINADO'
                        self.imprimir_ganador(mejor[0], mejor[1], mejor[2])
                else:
                    self.estado = 'PAUSA'

        self.pub_cmd.publish(cmd)

    def cambiar_a_fase_2(self):
        mejor = min(self.resultados, key=lambda x: x[2])
        self.get_logger().info(f"\n--- FASE 1 TERMINADA: Mejor Kp={mejor[0]} Ki={mejor[1]} ---")
        self.get_logger().info("Iniciando Fase 2: Zoom Decimal...")
        
        # Generar cuadrícula fina alrededor del mejor de la fase 1
        self.lista_pruebas = []
        for dk in [-10.0, -5.0, 0.0, 5.0, 10.0]:
            for di in [-2.5, 0.0, 2.5]:
                nk = max(0.0, mejor[0] + dk)
                ni = max(0.0, mejor[1] + di)
                self.lista_pruebas.append((nk, ni))
        
        self.lista_pruebas = list(set(self.lista_pruebas)) # Eliminar duplicados
        self.indice_prueba = 0
        self.resultados = []
        self.fase_actual = 2
        self.estado = 'PAUSA'

    def imprimir_ganador(self, kp, ki, err):
        self.get_logger().info("\n=======================================")
        self.get_logger().info("SINTONIZACIÓN V2 COMPLETADA")
        self.get_logger().info(f"Kp Óptimo: {kp}")
        self.get_logger().info(f"Ki Óptimo: {ki}")
        self.get_logger().info(f"Error Final: {err*1000:.2f} milímetros")
        self.get_logger().info("Usa estos valores en tu smart_parking.py")
        self.get_logger().info("=======================================\n")

def main():
    rclpy.init(); rclpy.spin(AutoTunerPI()); rclpy.shutdown()

if __name__ == '__main__':
    main()