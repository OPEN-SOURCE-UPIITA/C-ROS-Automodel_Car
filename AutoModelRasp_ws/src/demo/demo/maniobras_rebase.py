# maniobras_rebase.py

class ManiobrasRebase:
    def __init__(self, servo_centro=1500):
        self.servo_centro = servo_centro

    def logica_rebase_derecha(self, valor_servo_vision):
        dir_dc = 1
        speed_dc = 80
        dir_servo = 1700 # Forzamos servo a la derecha (Ejemplo)
        stop_lights = 0
        turn_signals = 1 # Luz direccional derecha encendida
        
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals
    
    def logica_rebase_izquierda(self, valor_servo_vision):
        dir_dc = 1
        speed_dc = 80
        dir_servo = 1300 # Forzamos servo a la izquierda (Ejemplo)
        stop_lights = 0
        turn_signals = 2 # Luz direccional izquierda encendida
        
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals