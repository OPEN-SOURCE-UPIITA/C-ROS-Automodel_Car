#!/usr/bin/env python3
"""
Nodo Maestro de Estacionamiento Autónomo ROS 2.
- Versión 100.0: Control de Velocidad en Lazo Cerrado (Encoder-Adaptive).
=======
╔══════════════════════════════════════════════════════════════════════════════╗
║        NODO ROS 2 — ESTACIONAMIENTO EN BATERÍA AUTÓNOMO  v3.0              ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Vehículo  : 44 cm largo × 20 cm ancho                                      ║
║  LIDAR     : CENTRADO en el carro (22 cm de frente y cola, 10 cm de lados)  ║
║  Espacio   : 30 cm ancho × 50 cm profundo, entre 2 cajas                    ║
║  Maniobra  : Batería, entrando de REVERSA                                   ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  MEJORAS vs v2.0                                                             ║
║  ┌─────────────────────────────────────────────────────────────────────┐    ║
║  │ 1. Filtro de postes del techo: enmascara las 4 columnas del carro   │    ║
║  │ 2. Búsqueda a velocidad lenta (PWM 45) para lectura estable         │    ║
║  │ 3. Frenada + confirmación al detectar apertura del hueco            │    ║
║  │ 4. Medición muy lenta (PWM 35) con odometría precisa                │    ║
║  │ 5. Frenada + validación al detectar el borde de la 2ª caja          │    ║
║  │ 6. Geometría recalculada para LIDAR centrado                        │    ║
║  │ 7. Histéresis en umbrales LIDAR (anti-rebotes)                      │    ║
║  │ 8. Corrección proporcional de yaw en REVERSA_RECTA                  │    ║
║  └─────────────────────────────────────────────────────────────────────┘    ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  GEOMETRÍA CLAVE (LIDAR centrado)                                            ║
║                                                                              ║
║    LIDAR_A_FRENTE = LARGO/2 = 22 cm                                         ║
║    LIDAR_A_COLA   = LARGO/2 = 22 cm                                         ║
║                                                                              ║
║    Postes del techo (esquinas del carro):                                    ║
║      distancia  = √(22² + 10²) ≈ 24.2 cm                                   ║
║      ángulos    = ±24.4° (frente) y ±155.6° (trasera)                       ║
║                                                                              ║
║    DIST_STOP cuando carro está completamente adentro:                        ║
║      espacio 50 cm - LIDAR_A_FRENTE 22 cm = 28 cm (ideal)                   ║
║      con margen 3 cm → DIST_STOP = 25 cm                                    ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math
import time # Necesario para calcular la velocidad real


# ══════════════════════════════════════════════════════════════════════════════
#  PARÁMETROS FÍSICOS Y DE CONTROL — ajusta aquí sin tocar la lógica
# ══════════════════════════════════════════════════════════════════════════════
class P:
    # ── Vehículo ────────────────────────────────────────────────────────────
    LARGO           = 0.44    # m
    ANCHO           = 0.20    # m
    TRACK           = 0.3210  # m  (separación entre ruedas, para odometría)
    TICKS_POR_M     = 4757.0  # ticks de encoder por metro

    # LIDAR centrado en el carro
    LIDAR_A_FRENTE  = LARGO / 2   # 0.22 m
    LIDAR_A_COLA    = LARGO / 2   # 0.22 m
    LIDAR_A_LADO    = ANCHO / 2   # 0.10 m

    # ── Postes del techo (4 esquinas) ───────────────────────────────────────
    # Con el LIDAR al centro, cada poste está a:
    #   distancia = √(LIDAR_A_FRENTE² + LIDAR_A_LADO²) = √(0.22²+0.10²) ≈ 0.242 m
    #   ángulos (convención ROS, 0=frente, CCW+):
    _d_poste = math.sqrt((LARGO/2)**2 + (ANCHO/2)**2)   # ≈ 0.242 m
    _a_esquina = math.degrees(math.atan2(ANCHO/2, LARGO/2))  # ≈ 24.4°

    # (ángulo_grados, distancia_nominal)
    POSTES = [
        ( _a_esquina,       _d_poste),  # frontal izquierdo
        (-_a_esquina,       _d_poste),  # frontal derecho
        ( 180 - _a_esquina, _d_poste),  # trasero izquierdo
        (-180 + _a_esquina, _d_poste),  # trasero derecho
    ]
    POSTE_VENTANA_ANG  = 5.0   # grados — zona ciega ± alrededor del poste
    POSTE_VENTANA_DIST = 0.10  # m     — filtra lecturas en [d-0.10, d+0.10]

    # ── Espacio de estacionamiento ──────────────────────────────────────────
    PROF_ESP  = 0.50   # m  (profundidad, perpendicular a la vía)
    ANCHO_ESP = 0.30   # m  (ancho del hueco)

    # ── Distancia de parada (lectura trasera cuando carro está adentro) ──────
    #   ideal = PROF_ESP - LIDAR_A_FRENTE = 0.50 - 0.22 = 0.28 m
    #   margen seguridad 3 cm → parar en 0.25 m
    DIST_STOP = 0.25   # m  ← DIST_STOP con LIDAR centrado

    # ── Sectores angulares del LIDAR ────────────────────────────────────────
    # Convención ROS estándar: 0°=frente, +90°=izquierda, -90°=derecha, ±180°=atrás
    ANG_DER       = -90.0   # grados — mirando a la DERECHA (hacia los huecos)
    ANG_TRAS      =  180.0  # grados — mirando hacia ATRÁS
    VENT_DER      =   18.0  # grados — ventana ±18° en el sector derecho
    VENT_TRAS     =   12.0  # grados — ventana ±12° en el sector trasero (estrecha)

    # ── Umbrales de detección (con histéresis) ──────────────────────────────
    UMBRAL_OBJ_ON   = 0.90   # m — dist_der < ON  → hay obstáculo (se activa)
    UMBRAL_OBJ_OFF  = 1.00   # m — dist_der > OFF → hueco libre   (se desactiva)
    MIN_HUECO       = 0.28   # m — tamaño mínimo del hueco aceptable
    MAX_DIST_BUSQ   = 5.0    # m — máx dist_der en búsqueda (descarta ruido lejano)

    # ── Velocidades diferenciadas ───────────────────────────────────────────
    PWM_BUSQUEDA    = 45     # lento al buscar el hueco — mejor resolución LIDAR
    PWM_MIDIENDO    = 35     # muy lento al medir el hueco
    PWM_MANIOBRA    = 65     # velocidad de maniobra de estacionamiento

    # ── Ciclos de pausa (a 20 Hz = 50 ms/ciclo) ────────────────────────────
    CICLOS_CONFIRMA_APERTURA = 12   # 0.6 s parado confirmando apertura del hueco
    CICLOS_CONFIRMA_CIERRE   = 15   # 0.75 s parado confirmando el borde final

    # ── Avance de posicionamiento ───────────────────────────────────────────
    # Al detectar la 2ª caja, el LIDAR está al nivel del borde interior.
    # La COLA está 22 cm MÁS ATRÁS → necesitamos avanzar ≥22 cm para que
    # la cola cruce el borde. Agregamos 8 cm de margen de maniobra.
    AVANCE_POS = 0.30   # m = LIDAR_A_COLA (0.22) + margen (0.08)

    # ── Seguridad durante maniobra ──────────────────────────────────────────
    PELIGRO_TRAS  = 0.20   # m — dist trasera < esto → rescate inmediato
    PELIGRO_LAT   = 0.07   # m — dist lateral  < esto → peligro de choque

    # ── Corrección proporcional de yaw (REVERSA_RECTA) ─────────────────────
    YAW_OBJ     = 87.0   # grados — ángulo que cuenta como "90° alcanzado"
    KP_YAW      = 1.5    # ganancia proporcional del servo para corrección
    MAX_CORR    = 100    # máxima corrección del servo en unidades PWM

    # ── Servo ───────────────────────────────────────────────────────────────
    SERVO_CENTRO  = 1500
    SERVO_DER_MAX = 1260   # máximo giro derecha (reversa)
    SERVO_IZQ_MAX = 1740   # máximo giro izquierda


# ══════════════════════════════════════════════════════════════════════════════
#  NODO PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════
class SmartParking(Node):

    # ──────────────────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__('smart_parking')
        
        self.pub_cmd = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data) 
        self.enc_sub = self.create_subscription(EncoderData, '/encoder_data', self.enc_cb, 10)
        
        # --- CONFIGURACIÓN FÍSICA Y CINEMÁTICA ---
        self.ticks_por_metro = 4757.0  
        self.ancho_via = 0.3210        
        
        # --- CONTROL DE VELOCIDAD ADAPTATIVA (PID PROPORCIONAL) ---
        self.vel_objetivo = 0.20       # Queremos ir siempre a 20 cm por segundo (0.20 m/s)
        self.vel_actual = 0.0          # Velocidad leída por los encoders
        self.pwm_base = 70.0           # PWM mínimo estimado para que el auto se mueva
        self.Kp = 120.0                # Qué tan agresivo compensa si va lento o rápido
        self.last_time = time.time()   # Reloj para el Delta de Tiempo
        
        # --- ACTUADORES ---
        self.DERECHA_REVERSA = 1260  
        self.IZQUIERDA_ADELANTE = 1740 
        self.CENTRO = 1500

        # --- VARIABLES DE CONTROL Y ODOMETRÍA ---
        self.estado = 'BUSCANDO_CARRO'
        self.pos_x = 0.0; self.pos_y = 0.0; self.yaw_actual = 0.0
        
        self.dist_frente_min = 10.0; self.dist_atras_min = 10.0
        self.dist_izq_min = 10.0; self.dist_der_min = 10.0 
        self.dist_carro_ref = 10.0 
        
        self.yaw_calle = 0.0 
        self.comando_dir = 1 
        self.x_ref = 0.0; self.y_ref = 0.0; self.p1_x = 0.0; self.p1_y = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🏎️ v100.0 - Velocidad Adaptativa Lazo Cerrado Activada.")

    def enc_cb(self, msg):
        # 1. Calculamos Odometría (Posición)
        signo = 1 if self.comando_dir == 1 else -1
        dist_lineal = ((abs(msg.vel_m1) + abs(msg.vel_m2)) / 2.0 / self.ticks_por_metro)
        ds_l = (abs(msg.vel_m1) / self.ticks_por_metro) * signo
        ds_r = (abs(msg.vel_m2) / self.ticks_por_metro) * signo
        self.yaw_actual += (ds_r - ds_l) / self.ancho_via
        self.pos_x += (dist_lineal * signo) * math.cos(self.yaw_actual)
        self.pos_y += (dist_lineal * signo) * math.sin(self.yaw_actual)

        # 2. Calculamos Velocidad Real (m/s)
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            self.vel_actual = dist_lineal / dt
        self.last_time = current_time

    def obtener_pwm_adaptativo(self):
        """Calcula el PWM exacto para mantener los 0.20 m/s sin importar la batería"""
        error_vel = self.vel_objetivo - self.vel_actual
        pwm_calculado = self.pwm_base + (error_vel * self.Kp)
        
        # Limitamos para que nunca se pase del 100% ni baje del límite de fricción (ej. 40%)
        pwm_seguro = max(40.0, min(95.0, pwm_calculado))
        return int(pwm_seguro)

    def obtener_minimo_sector(self, ranges, centro, apertura, limite_inferior=0.05, limite_superior=10.0):
        n = len(ranges)
        sector = []
        for i in range(centro - apertura, centro + apertura):
            idx = i % n 
            sector.append(ranges[idx])
        validos = [d for d in sector if limite_inferior < d < limite_superior and not math.isnan(d) and not math.isinf(d)]
        return min(validos) if validos else limite_superior

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0: return
        idx_frente = 0; idx_izq = int(n * 0.25); idx_atras = int(n * 0.50); idx_der = int(n * 0.75)
        self.dist_frente_min = self.obtener_minimo_sector(msg.ranges, idx_frente, 20)
        self.dist_atras_min  = self.obtener_minimo_sector(msg.ranges, idx_atras, 20)
        self.dist_izq_min    = self.obtener_minimo_sector(msg.ranges, idx_izq, 20)
        self.dist_der_min    = self.obtener_minimo_sector(msg.ranges, idx_der, 20)

    def control_loop(self):
        cmd = MotorCommand()
        cmd.turn_signals = 0; cmd.stop_lights = 0
        def diff_ang(a, b): return math.atan2(math.sin(a-b), math.cos(a-b))
        error_yaw = abs(math.degrees(diff_ang(self.yaw_actual, self.yaw_calle)))
        
        # Obtenemos el PWM ajustado de este instante
        pwm_dinamico = self.obtener_pwm_adaptativo()

        if self.estado not in ['ESTACIONADO']:
            self.get_logger().info(f"[{self.estado}] V:{self.vel_actual:.2f}m/s -> PWM:{pwm_dinamico} | D:{self.dist_der_min:.2f}m", throttle_duration_sec=0.5)

        # =================================================================
        # FASE 1: BÚSQUEDA DEL ANCHO DEL CAJÓN
        # =================================================================
        if self.estado == 'BUSCANDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 1 
            if self.dist_der_min < 1.50: 
                self.dist_carro_ref = self.dist_der_min 
                self.estado = 'SIGUIENDO_CARRO'

        elif self.estado == 'SIGUIENDO_CARRO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 1
            if self.dist_der_min < self.dist_carro_ref:
                self.dist_carro_ref = self.dist_der_min
                
            if self.dist_der_min > (self.dist_carro_ref + 0.30): 
                self.p1_x, self.p1_y = self.pos_x, self.pos_y
                self.estado = 'MIDIENDO_HUECO'

        elif self.estado == 'MIDIENDO_HUECO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 1
            dist_recorrida = math.hypot(self.pos_x - self.p1_x, self.pos_y - self.p1_y)
            
            if self.dist_der_min < (self.dist_carro_ref + 0.20) or dist_recorrida >= 0.35:
                if dist_recorrida < 0.25:
                    self.estado = 'BUSCANDO_CARRO' 
                else:
                    self.yaw_calle = self.yaw_actual
                    self.x_ref, self.y_ref = self.pos_x, self.pos_y
                    self.estado = 'AVANCE_POSICIONAMIENTO'

        elif self.estado == 'AVANCE_POSICIONAMIENTO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.CENTRO
            cmd.turn_signals = 3 
            if math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref) >= 0.25:
                self.estado = 'REVERSA_GIRANDO_90'

        # =================================================================
        # FASE 2: MANIOBRA
        # =================================================================
        elif self.estado == 'REVERSA_GIRANDO_90':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, pwm_dinamico, self.DERECHA_REVERSA
            cmd.turn_signals = 3
            
            if error_yaw >= 85.0:
                self.estado = 'REVERSA_FONDO_RECTO'
            elif self.dist_atras_min < 0.35 or self.dist_der_min < 0.12:
                self.x_ref, self.y_ref = self.pos_x, self.pos_y
                self.estado = 'ACOMODO_ADELANTE'

        elif self.estado == 'ACOMODO_ADELANTE':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 1, pwm_dinamico, self.IZQUIERDA_ADELANTE
            cmd.turn_signals = 3
            dist_avanzada = math.hypot(self.pos_x - self.x_ref, self.pos_y - self.y_ref)
            
            if dist_avanzada >= 0.20 or error_yaw >= 85.0 or self.dist_frente_min < 0.15:
                self.estado = 'REVERSA_GIRANDO_90'
                
        elif self.estado == 'REVERSA_FONDO_RECTO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 2, pwm_dinamico, self.CENTRO
            if self.dist_atras_min < 0.20:
                self.estado = 'ESTACIONADO'

        elif self.estado == 'ESTACIONADO':
            cmd.dir_dc, cmd.speed_dc, cmd.dir_servo = 0, 0, self.CENTRO
            cmd.stop_lights = 1; cmd.turn_signals = 3

        # Preparar y publicar
        self.comando_dir = int(cmd.dir_dc)
        cmd.dir_dc = int(cmd.dir_dc); cmd.speed_dc = int(cmd.speed_dc); cmd.dir_servo = int(cmd.dir_servo)
        cmd.stop_lights = int(cmd.stop_lights); cmd.turn_signals = int(cmd.turn_signals)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args); nodo = SmartParking()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally: nodo.destroy_node(); rclpy.shutdown()
=======

        # ── Comunicación ROS ─────────────────────────────────────────────────
        self.pub = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.create_subscription(LaserScan,   '/scan',         self._scan_cb, 10)
        self.create_subscription(EncoderData, '/encoder_data', self._enc_cb,  10)

        # ── Odometría ────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.yaw_calle = 0.0   # yaw guardado al momento de confirmar el hueco
        self._cmd_dir  = 1     # 1=adelante 2=reversa (signo para odometría)

        # ── LIDAR ────────────────────────────────────────────────────────────
        self.dist_der   = 10.0
        self.dist_atras = 10.0
        self._scan_ok   = False
        self._ang_min   = 0.0
        self._ang_inc   = 0.0
        self._n_rays    = 0
        self._hay_obst  = False   # estado de histéresis del obstáculo derecho

        # ── Máquina de estados ───────────────────────────────────────────────
        self.estado = 'ESPERANDO_LIDAR'

        # ── Referencias odométricas ──────────────────────────────────────────
        self.ref_x = 0.0;  self.ref_y = 0.0
        self.p1_x  = 0.0;  self.p1_y  = 0.0   # inicio del hueco detectado

        # ── Contador para estados de pausa ───────────────────────────────────
        self._pausa_cnt   = 0
        self._pausa_max   = 0
        self._pausa_sig   = ''

        # ── Distancia medida del hueco (guardada para el log) ────────────────
        self._tam_hueco   = 0.0

        self.create_timer(0.05, self._loop)   # 20 Hz

        self.get_logger().info(
            "🚗 SmartParking v3.0\n"
            f"   Carro  : {P.LARGO*100:.0f} × {P.ANCHO*100:.0f} cm\n"
            f"   Espacio: {P.ANCHO_ESP*100:.0f} × {P.PROF_ESP*100:.0f} cm\n"
            f"   LIDAR  : centrado — 4 postes filtrados automáticamente\n"
            f"   DIST_STOP: {P.DIST_STOP*100:.0f} cm"
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def _enc_cb(self, msg):
        """Odometría diferencial incremental."""
        signo = 1 if self._cmd_dir == 1 else -1
        ds_l = signo * abs(msg.vel_m1) / P.TICKS_POR_M
        ds_r = signo * abs(msg.vel_m2) / P.TICKS_POR_M
        ds   = (ds_l + ds_r) / 2.0
        self.yaw += (ds_r - ds_l) / P.TRACK
        self.x   += ds * math.cos(self.yaw)
        self.y   += ds * math.sin(self.yaw)

    def _scan_cb(self, msg):
        """Procesa el LaserScan: filtra postes y calcula distancias."""
        n = len(msg.ranges)
        if n == 0:
            return

        # Inicialización con parámetros reales del sensor
        if not self._scan_ok:
            self._n_rays  = n
            self._ang_min = msg.angle_min
            self._ang_inc = (msg.angle_max - msg.angle_min) / max(n - 1, 1)
            self._scan_ok = True
            self.get_logger().info(
                f"📡 LIDAR listo: {n} rayos  "
                f"[{math.degrees(msg.angle_min):.1f}° → "
                f"{math.degrees(msg.angle_max):.1f}°]  "
                f"res={math.degrees(self._ang_inc):.2f}°/rayo"
            )
            self.estado = 'BUSCANDO_CARRO'

        # ── 1. Crear copia mutable y aplicar filtro de postes ────────────────
        ranges_limpios = list(msg.ranges)
        self._filtrar_postes(ranges_limpios)

        # ── 2. Calcular distancias de interés ────────────────────────────────
        self.dist_der   = self._sector_min(ranges_limpios, P.ANG_DER,  P.VENT_DER,  max_d=P.MAX_DIST_BUSQ)
        self.dist_atras = self._sector_min(ranges_limpios, P.ANG_TRAS, P.VENT_TRAS, max_d=2.0)

        # ── 3. Actualizar histéresis del obstáculo derecho ───────────────────
        if self._hay_obst:
            # Se desactiva solo si supera el umbral superior
            if self.dist_der > P.UMBRAL_OBJ_OFF:
                self._hay_obst = False
        else:
            # Se activa solo si baja del umbral inferior
            if self.dist_der < P.UMBRAL_OBJ_ON:
                self._hay_obst = True

    # ══════════════════════════════════════════════════════════════════════════
    #  LIDAR — utilidades
    # ══════════════════════════════════════════════════════════════════════════

    def _ang_a_idx(self, ang_deg):
        """Convierte un ángulo (grados, convención ROS) a índice del array."""
        ang_rad = math.radians(ang_deg)
        # Normalizar al rango [angle_min, angle_min + 2π]
        while ang_rad < self._ang_min:
            ang_rad += 2 * math.pi
        while ang_rad > self._ang_min + 2 * math.pi:
            ang_rad -= 2 * math.pi
        return int(round((ang_rad - self._ang_min) / self._ang_inc)) % self._n_rays

    def _sector_min(self, ranges, ang_centro_deg, ventana_deg, max_d=5.0):
        """
        Distancia mínima válida en un sector angular.
        Usa angle_min y angle_increment reales (no índices fijos).
        """
        idx_c  = self._ang_a_idx(ang_centro_deg)
        idx_v  = int(round(math.radians(ventana_deg) / self._ang_inc))
        validos = []
        for i in range(idx_c - idx_v, idx_c + idx_v + 1):
            d = ranges[i % self._n_rays]
            if not (math.isnan(d) or math.isinf(d)) and 0.02 < d < max_d:
                validos.append(d)
        return min(validos) if validos else max_d

    def _filtrar_postes(self, ranges):
        """
        Enmascara (→ NaN) las lecturas que corresponden a los 4 postes
        del techo del carro.

        Cada poste produce un eco en un ángulo y distancia conocidos
        (calculados en la clase P desde las dimensiones del vehículo).
        Se enmascara una ventana de ±POSTE_VENTANA_ANG grados alrededor
        del ángulo nominal, SOLO si la distancia leída cae dentro del
        rango [d_nominal - VENTANA_DIST, d_nominal + VENTANA_DIST].
        Esto evita borrar lecturas de obstáculos reales que casualmente
        estén en el mismo ángulo pero a distancia diferente.
        """
        if not self._scan_ok:
            return

        for ang_deg, d_nom in P.POSTES:
            idx_c = self._ang_a_idx(ang_deg)
            vent  = int(round(math.radians(P.POSTE_VENTANA_ANG) / self._ang_inc))
            d_lo  = d_nom - P.POSTE_VENTANA_DIST
            d_hi  = d_nom + P.POSTE_VENTANA_DIST

            for i in range(idx_c - vent, idx_c + vent + 1):
                idx = i % self._n_rays
                d   = ranges[idx]
                if not (math.isnan(d) or math.isinf(d)) and d_lo < d < d_hi:
                    ranges[idx] = math.nan   # enmascarar — es un poste

    # ══════════════════════════════════════════════════════════════════════════
    #  UTILIDADES DE CONTROL
    # ══════════════════════════════════════════════════════════════════════════

    def _dist(self, x0, y0):
        return math.hypot(self.x - x0, self.y - y0)

    def _err_yaw_abs(self):
        """Error angular absoluto (grados) respecto al yaw de la calle."""
        d = math.atan2(math.sin(self.yaw - self.yaw_calle),
                       math.cos(self.yaw - self.yaw_calle))
        return abs(math.degrees(d))

    def _err_yaw_signed(self):
        """Error angular con signo (grados) respecto al yaw de la calle."""
        return math.degrees(
            math.atan2(math.sin(self.yaw - self.yaw_calle),
                       math.cos(self.yaw - self.yaw_calle))
        )

    def _set_ref(self):
        self.ref_x, self.ref_y = self.x, self.y

    def _pub(self, dir_dc, speed, servo, stop=0):
        m             = MotorCommand()
        m.dir_dc      = int(dir_dc)
        m.speed_dc    = int(speed)
        m.dir_servo   = int(servo)
        m.stop_lights = int(stop)
        self._cmd_dir = int(dir_dc)
        self.pub.publish(m)

    def _iniciar_pausa(self, ciclos, estado_siguiente):
        """
        Frena el carro y cuenta 'ciclos' antes de pasar al siguiente estado.
        Se usa para confirmar lecturas LIDAR antes de comprometerse.
        """
        self._pausa_cnt = 0
        self._pausa_max = ciclos
        self._pausa_sig = estado_siguiente

    # ══════════════════════════════════════════════════════════════════════════
    #  BUCLE DE CONTROL PRINCIPAL (20 Hz)
    # ══════════════════════════════════════════════════════════════════════════

    def _loop(self):
        if not self._scan_ok:
            return

        ey_abs    = self._err_yaw_abs()
        ey_signed = self._err_yaw_signed()

        self.get_logger().info(
            f"[{self.estado:<26}] "
            f"der={self.dist_der:.2f}m  "
            f"atras={self.dist_atras:.2f}m  "
            f"err={ey_abs:.1f}°  "
            f"pos=({self.x:.2f},{self.y:.2f})",
            throttle_duration_sec=0.25
        )

        # ════════════════════════════════════════════════════════════════════
        #  BUSCANDO_CARRO
        #  Avanza LENTO buscando el primer obstáculo a la derecha.
        #  Velocidad baja → el LIDAR tiene más tiempo de muestreo por posición.
        # ════════════════════════════════════════════════════════════════════
        if self.estado == 'BUSCANDO_CARRO':
            self._pub(1, P.PWM_BUSQUEDA, P.SERVO_CENTRO)
            if self._hay_obst:
                self.estado = 'BORDEANDO_CARRO'
                self.get_logger().info(
                    f"📦 Obstáculo detectado a {self.dist_der:.2f} m. "
                    "Siguiendo borde derecho..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  BORDEANDO_CARRO
        #  Sigue el borde del primer carro/caja hasta que el espacio se abra.
        #  Continúa lento para no pasarse del inicio del hueco.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'BORDEANDO_CARRO':
            self._pub(1, P.PWM_BUSQUEDA, P.SERVO_CENTRO)
            if not self._hay_obst:
                # ¡Posible apertura! Guardar posición y confirmar
                self.p1_x, self.p1_y = self.x, self.y
                self._iniciar_pausa(P.CICLOS_CONFIRMA_APERTURA, 'MIDIENDO_HUECO')
                self.estado = 'CONFIRMANDO_APERTURA'
                self.get_logger().info(
                    f"🔔 Apertura detectada en ({self.x:.2f},{self.y:.2f}). "
                    f"Confirmando {P.CICLOS_CONFIRMA_APERTURA} ciclos..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  CONFIRMANDO_APERTURA
        #  Carro DETENIDO. Espera N ciclos para asegurarse de que el hueco
        #  es real y no un rebote de lectura. Si durante la pausa vuelve a
        #  detectar obstáculo, era falso → volver a bordear.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'CONFIRMANDO_APERTURA':
            self._pub(0, 0, P.SERVO_CENTRO)   # quieto

            if self._hay_obst:
                # Era ruido — el hueco se cerró durante la pausa
                self.estado = 'BORDEANDO_CARRO'
                self.get_logger().warn("❌ Falsa apertura (ruido). Retomando borde...")
            else:
                self._pausa_cnt += 1
                if self._pausa_cnt >= self._pausa_max:
                    self.estado = self._pausa_sig
                    self.get_logger().info(
                        f"✅ Apertura real confirmada. "
                        f"Midiendo hueco despacio (PWM {P.PWM_MIDIENDO})..."
                    )

        # ════════════════════════════════════════════════════════════════════
        #  MIDIENDO_HUECO
        #  Avanza MUY DESPACIO midiendo la longitud del hueco con odometría.
        #  La referencia p1 ya quedó guardada al inicio del hueco.
        #  Termina cuando:
        #   (a) se detecta la 2ª caja → confirmar cierre
        #   (b) el hueco es demasiado largo (falso positivo / fin de zona)
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'MIDIENDO_HUECO':
            self._pub(1, P.PWM_MIDIENDO, P.SERVO_CENTRO)
            avance = self._dist(self.p1_x, self.p1_y)

            if self._hay_obst:
                # Borde de la 2ª caja detectado
                self._tam_hueco = avance
                self._set_ref()   # ref = posición exacta del borde de la 2ª caja

                if avance >= P.MIN_HUECO:
                    self._iniciar_pausa(P.CICLOS_CONFIRMA_CIERRE, 'POSICION_VALIDADA')
                    self.estado = 'CONFIRMANDO_CIERRE'
                    self.get_logger().info(
                        f"📏 2ª caja detectada. Hueco: {avance*100:.1f} cm. "
                        f"Confirmando cierre {P.CICLOS_CONFIRMA_CIERRE} ciclos..."
                    )
                else:
                    self.get_logger().warn(
                        f"❌ Hueco {avance*100:.1f} cm < mínimo "
                        f"{P.MIN_HUECO*100:.0f} cm. Descartado."
                    )
                    self.estado = 'BORDEANDO_CARRO'

            elif avance > 3.0:
                # Sin 2ª caja en 3 m → zona sin límite, reiniciar
                self.get_logger().warn(
                    "⚠️  No se encontró la 2ª caja en 3 m. Reiniciando búsqueda."
                )
                self.estado = 'BUSCANDO_CARRO'

        # ════════════════════════════════════════════════════════════════════
        #  CONFIRMANDO_CIERRE
        #  Carro DETENIDO en el borde de la 2ª caja.
        #  Confirma que el hueco es válido antes de iniciar la maniobra.
        #  Si el obstáculo desaparece (rebote), continúa midiendo.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'CONFIRMANDO_CIERRE':
            self._pub(0, 0, P.SERVO_CENTRO)   # quieto

            if not self._hay_obst:
                # La 2ª caja "desapareció" — era ruido, seguir midiendo
                self.estado = 'MIDIENDO_HUECO'
                self.get_logger().warn("❌ Falso cierre. Continuando medición...")
            else:
                self._pausa_cnt += 1
                if self._pausa_cnt >= self._pausa_max:
                    self.estado = self._pausa_sig

        # ════════════════════════════════════════════════════════════════════
        #  POSICION_VALIDADA
        #  El hueco tiene tamaño correcto y ambos bordes confirmados.
        #  Guardar el yaw de la calle para referencia de toda la maniobra.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'POSICION_VALIDADA':
            self.yaw_calle = self.yaw
            self.estado = 'AVANCE_POSICIONAMIENTO'
            self.get_logger().info(
                f"🎯 Hueco {self._tam_hueco*100:.1f} cm VALIDADO. "
                f"yaw_calle={math.degrees(self.yaw_calle):.1f}°. "
                f"Avanzando {P.AVANCE_POS*100:.0f} cm para posicionar cola..."
            )

        # ════════════════════════════════════════════════════════════════════
        #  AVANCE_POSICIONAMIENTO
        #  Avanza AVANCE_POS metros desde el borde de la 2ª caja, de modo que
        #  la cola del carro (22 cm atrás del LIDAR) haya rebasado la caja
        #  y tengamos radio libre para el giro de reversa.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'AVANCE_POSICIONAMIENTO':
            self._pub(1, P.PWM_MANIOBRA, P.SERVO_CENTRO)
            if self._dist(self.ref_x, self.ref_y) >= P.AVANCE_POS:
                self._set_ref()
                self.estado = 'REVERSA_GIRANDO'
                self.get_logger().info(
                    f"↩️  Cola posicionada. Iniciando reversa con giro derecho..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  REVERSA_GIRANDO
        #  Reversa con máximo giro a la derecha hasta alcanzar ~90°.
        #  Si hay obstáculo muy cerca de la cola → rescate inmediato.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'REVERSA_GIRANDO':
            self._pub(2, P.PWM_MANIOBRA, P.SERVO_DER_MAX)

            if self.dist_atras < P.PELIGRO_TRAS:
                self._set_ref()
                self.estado = 'RESCATE_ADELANTE'
                self.get_logger().warn(
                    f"⚠️  Cola a {self.dist_atras*100:.0f} cm. Rescate necesario."
                )
            elif ey_abs >= P.YAW_OBJ:
                self.estado = 'REVERSA_RECTA'
                self.get_logger().info(
                    f"📐 {ey_abs:.0f}° alcanzados. Entrando recto al espacio..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  REVERSA_RECTA
        #  Entra recto al hueco (perpendicular).
        #  Corrección proporcional de yaw para compensar la deriva del giro.
        #
        #  El servo se ajusta en función del error angular:
        #    error_yaw_signed > 0 → carro girado a la izquierda → corregir derecha
        #    error_yaw_signed < 0 → carro girado a la derecha  → corregir izquierda
        #
        #  Para en DIST_STOP = 25 cm de la pared (LIDAR_A_FRENTE = 22 cm + 3 cm de margen)
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'REVERSA_RECTA':
            corr  = int(max(-P.MAX_CORR, min(P.MAX_CORR, P.KP_YAW * ey_signed)))
            servo = max(P.SERVO_DER_MAX, min(P.SERVO_IZQ_MAX, P.SERVO_CENTRO - corr))
            self._pub(2, P.PWM_MANIOBRA, servo)

            if self.dist_atras < P.DIST_STOP:
                self.estado = 'ESTACIONADO'
                dist_cola = self.dist_atras - P.LIDAR_A_COLA
                self.get_logger().info(
                    f"🅿️  ¡ESTACIONADO!\n"
                    f"   LIDAR trasero  : {self.dist_atras*100:.0f} cm de la pared\n"
                    f"   Cola del carro : {dist_cola*100:.0f} cm de la pared\n"
                    f"   Hueco ocupado  : {self._tam_hueco*100:.1f} cm\n"
                    f"   Error angular  : {ey_abs:.1f}°"
                )

        # ════════════════════════════════════════════════════════════════════
        #  RESCATE_ADELANTE
        #  Avanza exactamente 20 cm (closed-loop por encoder) para ganar
        #  espacio libre y volver a intentar el giro.
        #  Solo usa odometría (no LIDAR) para no confundirse con el entorno.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'RESCATE_ADELANTE':
            self._pub(1, P.PWM_MANIOBRA, P.SERVO_CENTRO)
            if self._dist(self.ref_x, self.ref_y) >= 0.20:
                self.estado = 'REVERSA_GIRANDO'
                self.get_logger().info(
                    "↩️  Rescate 20 cm completado. Retomando giro..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  ESTACIONADO
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'ESTACIONADO':
            self._pub(0, 0, P.SERVO_CENTRO, stop=1)

        # ════════════════════════════════════════════════════════════════════
        #  ESPERANDO_LIDAR (estado inicial antes del primer mensaje scan)
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'ESPERANDO_LIDAR':
            self._pub(0, 0, P.SERVO_CENTRO)


# ══════════════════════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    rclpy.spin(SmartParking())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
