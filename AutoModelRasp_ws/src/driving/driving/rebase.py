# src/driving/driving/rebase.py
import numpy as np

class RebaseManiobra:
    """
    Gestor especializado para la maniobra de rebase.
    Maneja la lógica de cambio de carril, estabilización y retorno.
    """
    
    def __init__(self, servo_centro=1500, controlador_pd=None):
        # Configuración de servo
        self.servo_centro = servo_centro
        self.controlador_pd = controlador_pd  # Reutilizar el PD existente
        
        # Estados de la máquina de rebase
        self.ESTADO_INICIAL = 0
        self.ESTADO_VOLANTAZO_IZQ = 1
        self.ESTADO_ESTABILIZACION_IZQ = 2
        self.ESTADO_AVANCE_LIBRE = 3
        self.ESTADO_VOLANTAZO_DER = 4
        self.ESTADO_ESTABILIZACION_DER = 5
        self.ESTADO_FINAL = 6
        
        self.estado_actual = self.ESTADO_INICIAL
        self.tiempo_entrada_estado = 0.0
        
        # Variables de control
        self.carril_origen = None  # "IZQUIERDO" o "DERECHO"
        self.carril_destino = None
        self.tiempo_estabilizacion = 1.2  # segundos después del volantazo
        self.tiempo_espera_seguridad = 0.5  # antes de iniciar retorno
        
        # Parámetros ajustables
        self.volantazo_izq = 1150  # Valor del servo para girar izquierda
        self.volantazo_der = 1850  # Valor del servo para girar derecha
        self.volantazo_correccion = 1350  # Volantazo suave para estabilizar
        self.angulo_recto = 1500
        
        # Umbrales del LiDAR (en metros)
        self.distancia_segura = 1.5  # Distancia a la que consideramos que el obstáculo está "lejos"
        self.distancia_obstaculo = 0.5  # Distancia que activa la alerta original
        
        # Memoria para el PD durante la maniobra
        self.error_previo_guardado = None
        
    def iniciar_rebase(self, carril_actual, timestamp):
        """
        Inicia la maniobra de rebase.
        Guarda el carril de origen y prepara la máquina de estados.
        """
        if carril_actual not in ["IZQUIERDO", "DERECHO"]:
            # Si no detecta carril, asumir derecho por defecto
            carril_actual = "DERECHO"
            
        self.carril_origen = carril_actual
        
        # Determinar carril destino (el opuesto)
        if carril_actual == "IZQUIERDO":
            self.carril_destino = "DERECHO"
            self.volantazo_principal = self.volantazo_der
            self.volantazo_estabilizacion = self.volantazo_der - 200
        else:  # DERECHO
            self.carril_destino = "IZQUIERDO"
            self.volantazo_principal = self.volantazo_izq
            self.volantazo_estabilizacion = self.volantazo_izq + 200
            
        self.estado_actual = self.ESTADO_VOLANTAZO_IZQ
        self.tiempo_entrada_estado = timestamp
        
        # Guardar el error previo del PD para restaurarlo después
        if self.controlador_pd:
            self.error_previo_guardado = self.controlador_pd.error_previo
            
        return True
    
    def actualizar(self, dt_estado, timestamp, distancia_obstaculo, carril_actual, error_carril):
        """
        Actualiza la máquina de estados del rebase.
        Retorna: (velocidad, servo, terminado)
        """
        vel_cmd = 80  # Velocidad constante durante el rebase
        servo_cmd = self.angulo_recto
        terminado = False
        
        tiempo_en_estado = timestamp - self.tiempo_entrada_estado
        
        # ========== ESTADO 1: VOLANTAZO INICIAL ==========
        if self.estado_actual == self.ESTADO_VOLANTAZO_IZQ:
            # Aplicar volantazo fuerte hacia el carril destino
            servo_cmd = self.volantazo_principal
            vel_cmd = 70  # Reducir velocidad durante el volantazo
            
            # Dar tiempo para que el volantazo tenga efecto
            if tiempo_en_estado > 0.4:
                self.estado_actual = self.ESTADO_ESTABILIZACION_IZQ
                self.tiempo_entrada_estado = timestamp
                
        # ========== ESTADO 2: ESTABILIZACIÓN EN CARRIL NUEVO ==========
        elif self.estado_actual == self.ESTADO_ESTABILIZACION_IZQ:
            # Volantazo suave para ayudar a estabilizar
            servo_cmd = self.volantazo_estabilizacion
            vel_cmd = 75
            
            # Después de estabilizar, pasar al avance libre
            if tiempo_en_estado > self.tiempo_estabilizacion:
                self.estado_actual = self.ESTADO_AVANCE_LIBRE
                self.tiempo_entrada_estado = timestamp
                
        # ========== ESTADO 3: AVANCE LIBRE (monitoreando LiDAR) ==========
        elif self.estado_actual == self.ESTADO_AVANCE_LIBRE:
            # Usar el PD normal para mantener el carril
            if self.controlador_pd and error_carril is not None:
                # Usar el controlador PD con los parámetros actuales
                # Para evitar sobreescribir, usamos directamente el método
                servo_cmd = self.controlador_pd.calcular_direccion_pd(
                    error_carril, dt_estado, 1500.0, 200.0, 25
                )
            else:
                servo_cmd = self.angulo_recto
                
            vel_cmd = 85  # Velocidad normal de rebase
            
            # Verificar condiciones para regresar:
            # 1. El obstáculo ya no está cerca (superado)
            # 2. O estamos demasiado cerca (abortar por seguridad)
            # 3. O ha pasado mucho tiempo (máximo 8 segundos de rebase)
            
            obstaculo_superado = distancia_obstaculo > self.distancia_segura
            tiempo_maximo = tiempo_en_estado > 8.0
            
            if obstaculo_superado or tiempo_maximo:
                # Pequeña espera antes de iniciar retorno
                if tiempo_en_estado > self.tiempo_espera_seguridad:
                    self.estado_actual = self.ESTADO_VOLANTAZO_DER
                    self.tiempo_entrada_estado = timestamp
                    
        # ========== ESTADO 4: VOLANTAZO DE RETORNO ==========
        elif self.estado_actual == self.ESTADO_VOLANTAZO_DER:
            # Volantazo hacia el carril original
            if self.carril_origen == "IZQUIERDO":
                servo_cmd = self.volantazo_izq
            else:
                servo_cmd = self.volantazo_der
            vel_cmd = 70
            
            if tiempo_en_estado > 0.4:
                self.estado_actual = self.ESTADO_ESTABILIZACION_DER
                self.tiempo_entrada_estado = timestamp
                
        # ========== ESTADO 5: ESTABILIZACIÓN FINAL ==========
        elif self.estado_actual == self.ESTADO_ESTABILIZACION_DER:
            # Volantazo suave de corrección
            if self.carril_origen == "IZQUIERDO":
                servo_cmd = self.volantazo_izq + 200
            else:
                servo_cmd = self.volantazo_der - 200
            vel_cmd = 75
            
            if tiempo_en_estado > self.tiempo_estabilizacion:
                self.estado_actual = self.ESTADO_FINAL
                self.tiempo_entrada_estado = timestamp
                
        # ========== ESTADO 6: FINALIZADO ==========
        elif self.estado_actual == self.ESTADO_FINAL:
            terminado = True
            # Restaurar el PD
            if self.controlador_pd and self.error_previo_guardado is not None:
                self.controlador_pd.error_previo = self.error_previo_guardado
                
        return vel_cmd, servo_cmd, terminado
    
    def obtener_estado_descripcion(self):
        """Para debugging"""
        estados = {
            self.ESTADO_INICIAL: "INICIAL",
            self.ESTADO_VOLANTAZO_IZQ: "VOLANTAZO_INICIAL",
            self.ESTADO_ESTABILIZACION_IZQ: "ESTABILIZANDO",
            self.ESTADO_AVANCE_LIBRE: "AVANZANDO",
            self.ESTADO_VOLANTAZO_DER: "RETORNANDO",
            self.ESTADO_ESTABILIZACION_DER: "ESTABILIZANDO_RETORNO",
            self.ESTADO_FINAL: "FINAL"
        }
        return estados.get(self.estado_actual, "DESCONOCIDO")
    
    def esta_rebasando(self):
        """Indica si todavía estamos en medio del rebase"""
        return self.estado_actual not in [self.ESTADO_INICIAL, self.ESTADO_FINAL]
