# logica_maniobras.py

class ManiobrasAutonomo:
    def __init__(self):
        # Memoria interna para el rebase
        self.fase_rebase = 0
        self.tiempo_fase_rebase = 0.0

    def ejecutar_logica_stop(self, dt_estado, tiempo_detenido):
        """
        Mantiene el auto detenido por un tiempo determinado.
        """
        vel_cmd = 0
        servo_cmd = 1500 # Dirección recta
        terminado = False

        if dt_estado >= tiempo_detenido:
            terminado = True # Ya pasó el tiempo, terminamos la maniobra
            
        return vel_cmd, servo_cmd, terminado

    def ejecutar_logica_cruce(self, dt_estado, tiempo_espera):
        """
        Similar al Stop, pero preparado por si luego quieren añadir
        lógica de esperar a que el peatón pase.
        """
        vel_cmd = 0
        servo_cmd = 1500
        terminado = False

        if dt_estado >= tiempo_espera:
            terminado = True
            
        return vel_cmd, servo_cmd, terminado

    def ejecutar_logica_rebase(self, dt_estado, carril_actual, error_carril):
        """
        Máquina de estados interna para la maniobra de rebase.
        Fase 0: Dar volantazo a la izquierda hasta detectar el carril izquierdo.
        Fase 1: Avanzar recto por el carril izquierdo unos segundos.
        Fase 2: Regresar al carril derecho.
        """
        vel_cmd = 80 # Velocidad constante de rebase
        servo_cmd = 1500
        terminado = False

        # FASE 0: Salir al carril izquierdo
        if self.fase_rebase == 0:
            servo_cmd = 1100 # Girar a tope a la izquierda (ajustar según tu servo)
            # Si el detector multicarril ya nos dice que estamos en el IZQUIERDO
            # o si ya pasaron más de 2 segundos (por seguridad), pasamos de fase.
            if carril_actual == "IZQUIERDO" or dt_estado > 2.0:
                self.fase_rebase = 1
                self.tiempo_fase_rebase = dt_estado # Guardamos el tiempo en que cambiamos

        # FASE 1: Avanzar por el carril izquierdo
        elif self.fase_rebase == 1:
            # Aquí podríamos usar un PD secundario, pero por ahora vamos recto
            servo_cmd = 1500 
            dt_fase = dt_estado - self.tiempo_fase_rebase
            
            # Rebasamos durante 3 segundos
            if dt_fase > 3.0:
                self.fase_rebase = 2
                self.tiempo_fase_rebase = dt_estado

        # FASE 2: Regresar al carril derecho
        elif self.fase_rebase == 2:
            servo_cmd = 1900 # Girar a la derecha para regresar
            dt_fase = dt_estado - self.tiempo_fase_rebase
            
            # Si ya detectamos el carril derecho o pasó mucho tiempo, terminamos
            if carril_actual == "DERECHO" or dt_fase > 2.5:
                self.fase_rebase = 0 # Reiniciamos para el próximo rebase
                terminado = True

        return vel_cmd, servo_cmd, terminado