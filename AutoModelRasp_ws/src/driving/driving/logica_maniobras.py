# logica_maniobras.py

# Importar la nueva clase Rebase
from driving.rebase import RebaseManiobra

class ManiobrasAutonomo:
    def __init__(self, controlador_pd=None):
        # Memoria interna para el rebase
        self.fase_rebase = 0
        self.tiempo_fase_rebase = 0.0
        
        # Instanciar la nueva lógica de rebase
        self.rebase = RebaseManiobra(servo_centro=1500, controlador_pd=controlador_pd)
        
        # Variables para el LiDAR (recibir desde el nodo)
        self.distancias_lidar = [9.99, 9.99, 9.99, 9.99, 9.99]  # Valores por defecto (INF)
        
        # Bandera para saber si estamos en rebase activo
        self.en_rebase = False

    def actualizar_distancias_lidar(self, distancias):
        """
        Actualiza las distancias de las 5 franjas del LiDAR.
        distancias: lista de 5 floats con distancias en metros
        """
        if distancias and len(distancias) >= 5:
            self.distancias_lidar = distancias

    def actualizar_parametros_rebase(self, params_dict):
        """
        Actualiza los parámetros configurables del rebase desde RQT.
        """
        self.rebase.actualizar_parametros(params_dict)

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
        
        Versión mejorada con:
        - Contravolantazo para estabilización
        - Detección de coche de frente por LiDAR
        - Regreso suave al carril original
        """
        
        # Si es la primera vez que entramos al rebase, inicializar
        if not self.en_rebase:
            self.rebase.iniciar_rebase(carril_actual, dt_estado)
            self.en_rebase = True
            
        # Ejecutar la fase actual del rebase
        servo_cmd, vel_cmd, terminado = self.rebase.actualizar(
        #servo_cmd, vel_cmd, terminado, abortar = self.rebase.actualizar(
            dt_estado, 
            dt_estado, 
            carril_actual, 
            self.distancias_lidar[2],
            error_carril
        )
        
        """
        # Si hay aborto por coche de frente, regresamos inmediatamente
        if abortar:
            self.en_rebase = False
            terminado = True
            # Enviamos comando de regreso de emergencia
            if carril_actual == "IZQUIERDO":
                servo_cmd = 1900  # Regresar a derecha rápido
            else:
                servo_cmd = 1100  # Regresar a izquierda rápido
            vel_cmd = 70  # Reducir velocidad por seguridad

        """
            
        # Si la maniobra terminó, reiniciamos la bandera
        if terminado:
            self.en_rebase = False
            
        return vel_cmd, servo_cmd, terminado
