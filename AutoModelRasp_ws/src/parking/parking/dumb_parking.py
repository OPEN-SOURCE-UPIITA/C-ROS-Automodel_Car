#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand

class SmartParkingHybrid(Node):
    def __init__(self):
        super().__init__('smart_parking_hybrid')
        
        # Publicador de comandos para el motor
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        
        # --- ACTUADORES ---
        self.DERECHA = 1240  
        self.IZQUIERDA = 1740 
        self.CENTRO = 1500
        
        # --- VARIABLES PARA EL LAZO ABIERTO (TIEMPOS) ---
        self.paso_actual = 0
        self.tiempo_inicio_paso = None # Se inicializa en el primer ciclo
        
        # Formato de secuencia: (Dirección_DC, PWM, Dirección_Servo, Segundos, Descripción)
        # dir_dc: 0=Stop, 1=Adelante, 2=Reversa
        self.secuencia = [
            (1, 65, self.CENTRO,    2.0, "1. Avanzar recto"),
            (1, 65, self.IZQUIERDA, 0.5, "2. Agarrar Curva"),
            (1, 65, self.DERECHA,   1.0, "3. En Curva"),
            (1, 65, self.CENTRO,    1.0, "4. Avance Recto"),
            (0,  0, self.CENTRO,    0.2, "5. PAUSA"),
            (0,  0, self.IZQUIERDA, 0.8, "6. PAUSA"),
            (1, 65, self.IZQUIERDA, 0.8, "7. Abriendo a la izquierda"),
            (0,  0, self.CENTRO,    0.2, "8. PAUSA"),
            (0,  0, self.DERECHA,   0.8, "9. PAUSA"),
            (2, 65, self.DERECHA,   2.0, "10. Reacomodo Reversa"),
            (0,  0, self.CENTRO,    0.5, "11. PAUSA"),
            (2, 65, self.CENTRO,    1.0, "12. Reversa Final"),
            (0,  0, self.CENTRO,    999, "10. ESTACIONADO (FIN)")
        ]

        self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        cmd = MotorCommand()
        ahora = self.get_clock().now().nanoseconds / 1e9

        # Iniciar el cronómetro exactamente en la primera ejecución del loop
        if self.tiempo_inicio_paso is None:
            self.tiempo_inicio_paso = ahora

        tiempo_transcurrido = ahora - self.tiempo_inicio_paso
        
        # Extraer el comando del arreglo
        dir_dc, speed, servo, duracion, desc = self.secuencia[self.paso_actual]
        
        self.get_logger().info(f"[{self.paso_actual}] {desc} | {tiempo_transcurrido:.1f}s / {duracion}s", throttle_duration_sec=0.5)
        
        # ASIGNACIÓN ESTRICTA RESPETANDO TU MENSAJE
        cmd.dir_dc = int(dir_dc)       # int8 (0, 1, 2)
        cmd.speed_dc = int(speed)      # int8 (0-100)
        cmd.dir_servo = int(servo)     # int16 (1110-1740)
        cmd.stop_lights = 0            # int8 (0=Apagado)
        cmd.turn_signals = 0           # int8 (0=Apagado)
        
        # Condición de avance de paso
        if tiempo_transcurrido >= duracion:
            if self.paso_actual < len(self.secuencia) - 1:
                self.paso_actual += 1
                self.tiempo_inicio_paso = ahora 
            else:
                # Si ya terminó la secuencia, se asegura de detenerse
                cmd.dir_dc = 0
                cmd.speed_dc = 0
                cmd.stop_lights = 1 # Prender las luces de freno al terminar por estética/seguridad

        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(SmartParkingHybrid())
    rclpy.shutdown()

if __name__ == '__main__':
    main()