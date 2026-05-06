# maniobras_seniales.py

class ManiobrasSeniales:
    def __init__(self, servo_centro=1500):
        self.servo_centro = servo_centro

    # FORMATO DE RETORNO PARA TODAS LAS FUNCIONES:
    # return (dir_dc, speed_dc, dir_servo, stop_lights, turn_signals)
    # dir_dc: 0=Stop, 1=Adelante, 2=Atrás
    # speed_dc: 0-100
    # dir_servo: 1110-1740 (1500=Centro)
    # stop_lights: 0=Apagado, 1=Encendido
    # turn_signals: 0=Apagado, 1=Derecha, 2=Izquierda, 3=Intermitentes

    def logica_normal(self, valor_servo_vision):
        dir_dc = 1
        speed_dc = 80
        dir_servo = valor_servo_vision
        stop_lights = 0
        turn_signals = 0
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals

    def logica_stop(self, dt_estado):
        dir_dc = 0
        speed_dc = 0
        dir_servo = self.servo_centro
        stop_lights = 1  # Encendemos luces de freno
        turn_signals = 0
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals

    def logica_zona_escolar(self, valor_servo_vision):
        dir_dc = 1
        speed_dc = 60 # Reducimos velocidad
        dir_servo = valor_servo_vision
        stop_lights = 0
        turn_signals = 3 # Encendemos intermitentes de precaución
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals

    def logica_giro(self, dt_estado, direccion="izquierda"):
        dir_dc = 1
        speed_dc = 70
        stop_lights = 0
        
        if direccion == "izquierda":
            dir_servo = 1200 # Rango 1110-1740
            turn_signals = 2 # Direccional izquierda
        else:
            dir_servo = 1700
            turn_signals = 1 # Direccional derecha
            
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals

    def logica_derrapante(self, dt_estado):
        dir_dc = 1
        speed_dc = 80
        stop_lights = 0
        turn_signals = 3 # Intermitentes por peligro
        
        # Oscilar el servo cada 0.5 segundos usando el tiempo en el estado
        if int(dt_estado * 2) % 2 == 0:
            dir_servo = 1700 # Derecha
        else:
            dir_servo = 1300 # Izquierda
            
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals

    def logica_100kmh(self, valor_servo_vision):
        dir_dc = 1
        speed_dc = 100
        dir_servo = valor_servo_vision
        stop_lights = 0
        turn_signals = 0
        return dir_dc, speed_dc, dir_servo, stop_lights, turn_signals