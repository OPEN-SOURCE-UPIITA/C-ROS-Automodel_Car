# Archivo: controlador_pd.py
import numpy as np

class ControladorAutomodel:
    def __init__(self, servo_centro=1500):
        # Memorias internas del controlador
        self.servo_centro = servo_centro
        self.last_servo_value = self.servo_centro
        self.error_previo = 0.0

    def calcular_direccion_pd(self, error_actual, dt, kp, kd, max_step_base):
        """Calcula el comando del servo usando PD y suavizado (Slew Rate)"""
        
        # 1. Control PD clásico
        P = error_actual * kp
        D = kd * (error_actual - self.error_previo) / dt
        self.error_previo = error_actual

        target_servo = self.servo_centro - (P + D)
        target_servo = int(np.clip(target_servo, 1110, 1740))

        # 2. Slew Rate (Suavizado mecánico del servo)
        max_step = max_step_base + int(abs(error_actual) * 20)
        diff = target_servo - self.last_servo_value
        
        if abs(diff) > max_step:
            nuevo_servo = self.last_servo_value + np.sign(diff) * max_step
        else:
            nuevo_servo = target_servo
            
        self.last_servo_value = nuevo_servo
        return int(nuevo_servo)

    def calcular_velocidad_adaptativa(self, error_actual, vel_base, vel_min, factor_reduccion):
        """Reduce la velocidad en las curvas basándose en el error (Anti-Deadband)"""
        speed_sugerida = vel_base - (abs(error_actual) * factor_reduccion)
        speed_calculada = int(max(vel_min, min(speed_sugerida, vel_base)))
        return speed_calculada