#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
from sensor_msgs.msg import LaserScan
import math
from collections import deque


# ══════════════════════════════════════════════════════════════════════════════
#  PARÁMETROS
# ══════════════════════════════════════════════════════════════════════════════
class P:

    # ── Físicos del vehículo ─────────────────────────────────────────────────
    LARGO        = 0.44     # m
    ANCHO        = 0.20     # m
    LIDAR_REAR   = 0.22     # m — LIDAR al bumper trasero (LARGO / 2)
    LIDAR_FRONT  = 0.22     # m — LIDAR al bumper frontal (LARGO / 2)
    LIDAR_SIDE   = 0.10     # m — LIDAR al borde derecho  (ANCHO / 2)
    TRACK        = 0.3210   # m — separación entre ruedas (odometría)
    TICKS_POR_M  = 4757.0   # ticks de encoder por metro

    # ── Espacio de estacionamiento ───────────────────────────────────────────
    PROF_ESP     = 0.60     # m — profundidad del hueco
    ANCHO_ESP    = 0.30     # m — ancho del hueco
    MIN_HUECO    = 0.25     # m — longitud mínima aceptable (odometría)
    MAX_BUSQUEDA = 3.00     # m — distancia máxima sin encontrar 2ª caja

    # ── Distancia de parada ──────────────────────────────────────────────────
    # Carro completamente adentro, bumper trasero a ~3 cm de la pared:
    #   LIDAR (centrado, 22 cm de la cola) lee: 50 - 22 - 3 = 25 cm de la pared
    #   Se detiene en 25 cm (ya incluye margen)
    DIST_STOP    = 0.25     # m — distancia LIDAR trasero → pared del fondo

    # ── Sector lateral del LIDAR ─────────────────────────────────────────────
    # Convención ROS: 0° = frente, -90° = derecha, +90° = izq, ±180° = atrás.
    # Si tu LIDAR está rotado, ajusta ANG_DER.
    ANG_DER      = -90.0    # grados — dirección "derecha" del robot
    VENT_DER     =  20.0    # grados ± → sector total de 40° para el promedio
    ANG_TRAS     =  180.0   # grados — dirección "atrás" del robot
    VENT_TRAS    =   12.0   # grados ± (estrecha, para no leer las ruedas)

    # ── Detección por DELTA ──────────────────────────────────────────────────
    # Se mantiene un buffer de los últimos DELTA_N promedios de sector.
    # El delta se calcula como:
    #   delta = media_reciente - media_anterior
    # donde "reciente" y "anterior" son mitades iguales del buffer.
    #
    # DELTA_ABRIR: subida de distancia → el hueco se abrió (dist sube)
    # DELTA_CERRAR: bajada de distancia → apareció un objeto (dist baja)
    #
    # ► Calibración en pista:
    #   Empuja el carro a mano pasando por el hueco y ve el log de `avg_der`.
    #   La diferencia entre "junto a la caja" y "frente al hueco" es el delta real.
    #   Pon DELTA_ABRIR = 70% de esa diferencia.
    #   Pon DELTA_CERRAR = 70% de la caída al llegar a la 2ª caja.
    DELTA_N      = 10       # número de muestras en el buffer (10 × 50ms = 500ms)
    DELTA_ABRIR  =  0.30    # m — subida mínima para detectar apertura de hueco
    DELTA_CERRAR =  0.25    # m — caída mínima para detectar objeto (1ª/2ª caja)

    # ── Filtrado de postes del techo ─────────────────────────────────────────
    POST_DIST_MAX = 0.30    # m — lectura ≤ esto puede ser un poste
    POST_N_SCANS  = 40      # scans en quieto para calibrar (~2 s a 20 Hz)
    POST_FRAC     = 0.70    # fracción de scans cortos para confirmar poste

    # ── Seguridad durante reversa ────────────────────────────────────────────
    PELIGRO_TRAS = 0.30     # m — activa rescate si dist_atras < esto
    PELIGRO_LAT  = 0.14     # m — alerta de colisión lateral

    # ── Avance de posicionamiento ────────────────────────────────────────────
    # Al detectar la 2ª caja, el LIDAR está al nivel del borde interior.
    # La cola está LIDAR_REAR (22 cm) más atrás del LIDAR.
    # AVANCE_POS = LIDAR_REAR + margen de giro = 0.22 + 0.15 = 0.37 m
    AVANCE_POS   = 0.37     # m

    # ── Corrección de yaw en reversa recta ───────────────────────────────────
    YAW_OBJ      = 87.0     # grados — "90°" alcanzados
    KP_YAW       =  1.2     # ganancia proporcional
    MAX_CORR     = 80       # máxima corrección servo (pwm units)

    # ── Velocidades (todas ≥ 65) ─────────────────────────────────────────────
    PWM_NORMAL   = 72       # búsqueda y maniobra de reversa
    PWM_LENTO    = 67       # acercamiento al hueco (lectura estable)
    PWM_CREEP    = 65       # medición del hueco (velocidad mínima estable)

    # ── Servo ────────────────────────────────────────────────────────────────
    SERVO_CENTRO  = 1500
    SERVO_DER_REV = 1260    # máximo giro derecha en reversa

    # ── Pausa de confirmación en los bordes del hueco ────────────────────────
    PAUSA_SEG    = 0.40     # segundos en quieto al detectar inicio/fin del hueco


# ══════════════════════════════════════════════════════════════════════════════
#  NODO PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════
class SmartParking(Node):

    def __init__(self):
        super().__init__('smart_parking')

        self.pub = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.create_subscription(LaserScan,   '/scan',         self._scan_cb, 10)
        self.create_subscription(EncoderData, '/encoder_data', self._enc_cb,  10)

        # ── Odometría ────────────────────────────────────────────────────────
        self.x         = 0.0
        self.y         = 0.0
        self.yaw       = 0.0
        self.yaw_calle = 0.0
        self._dir      = 0          # 1=adelante, 2=reversa, 0=parado

        # ── Distancias actuales ───────────────────────────────────────────────
        self.avg_der   = 10.0       # PROMEDIO del sector derecho (m)
        self.dist_atras = 10.0      # mín del sector trasero (m)

        # ── LIDAR — metadatos ────────────────────────────────────────────────
        self._ang_min = 0.0
        self._ang_inc = 0.0
        self._n       = 0
        self._scan_ok = False

        # ── Filtro de postes ─────────────────────────────────────────────────
        self._post_mask  = None     # bool por rayo: True = poste
        self._calib_buf  = []

        # ── Buffer de delta ──────────────────────────────────────────────────
        # Almacena los últimos DELTA_N valores de avg_der.
        # El delta se calcula como:  ventana_nueva - ventana_vieja
        self._der_buf    = deque(maxlen=P.DELTA_N)

        # ── Referencias odométricas ──────────────────────────────────────────
        self.ref_x       = 0.0
        self.ref_y       = 0.0
        self.p1_x        = 0.0     # inicio del hueco
        self.p1_y        = 0.0
        self._hueco_long = 0.0

        # ── Temporizador de pausa ────────────────────────────────────────────
        self._t_pausa = None

        # ── Estados ──────────────────────────────────────────────────────────
        self.estado = 'ESPERANDO_LIDAR'

        self.create_timer(0.05, self._loop)   # 20 Hz

        self.get_logger().info(
            f"🚗 SmartParking v4.0 | "
            f"LIDAR centrado (rear={P.LIDAR_REAR*100:.0f} cm) | "
            f"DELTA_ABRIR={P.DELTA_ABRIR:.2f} m  DELTA_CERRAR={P.DELTA_CERRAR:.2f} m | "
            f"PWM_CREEP={P.PWM_CREEP}"
        )

    # ──────────────────────────────────────────────────────────────────────────
    #  Odometría diferencial
    # ──────────────────────────────────────────────────────────────────────────
    def _enc_cb(self, msg):
        if self._dir == 0:
            return
        s  = 1 if self._dir == 1 else -1
        dl = s * abs(msg.vel_m1) / P.TICKS_POR_M
        dr = s * abs(msg.vel_m2) / P.TICKS_POR_M
        ds = (dl + dr) / 2.0
        self.yaw += (dr - dl) / P.TRACK
        self.x   += ds * math.cos(self.yaw)
        self.y   += ds * math.sin(self.yaw)

    # ──────────────────────────────────────────────────────────────────────────
    #  Callback del LIDAR
    # ──────────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return

        if not self._scan_ok:
            self._n       = n
            self._ang_min = msg.angle_min
            self._ang_inc = (msg.angle_max - msg.angle_min) / max(n - 1, 1)
            self._scan_ok = True
            self.estado   = 'CALIBRANDO'
            self.get_logger().info(
                f"LIDAR: {n} rayos "
                f"[{math.degrees(msg.angle_min):.0f}° → "
                f"{math.degrees(msg.angle_max):.0f}°] "
                f"inc={math.degrees(self._ang_inc):.2f}°/rayo"
            )

        if self.estado == 'CALIBRANDO':
            self._calib_buf.append(list(msg.ranges))
            if len(self._calib_buf) >= P.POST_N_SCANS:
                self._calibrar_postes()
            return

        # ── Lecturas activas ──────────────────────────────────────────────────
        nueva_der   = self._sector_avg(msg.ranges, P.ANG_DER,  P.VENT_DER,  10.0)
        self.dist_atras = self._sector_min(msg.ranges, P.ANG_TRAS, P.VENT_TRAS,  2.0)

        self.avg_der = nueva_der
        self._der_buf.append(nueva_der)

    # ──────────────────────────────────────────────────────────────────────────
    #  Calibración de postes del techo
    # ──────────────────────────────────────────────────────────────────────────
    def _calibrar_postes(self):
        n = self._n
        self._post_mask = [False] * n

        for i in range(n):
            vals = [s[i] for s in self._calib_buf
                    if not math.isnan(s[i]) and not math.isinf(s[i])]
            if not vals:
                self._post_mask[i] = True
                continue
            frac = sum(1 for d in vals if d < P.POST_DIST_MAX) / len(vals)
            self._post_mask[i] = (frac >= P.POST_FRAC)

        n_post = sum(self._post_mask)
        self.get_logger().info(
            f"Calibración: {n_post}/{n} rayos de postes enmascarados. "
            f"{n - n_post} rayos activos."
        )
        if n_post > n * 0.70:
            self.get_logger().warn(
                "+70% rayos enmascarados. Verifica POST_DIST_MAX "
                "y que el robot estuviera quieto."
            )
        self._calib_buf.clear()
        self.estado = 'BUSCANDO_CARRO'

    # ──────────────────────────────────────────────────────────────────────────
    #  PROMEDIO de sector angular (ignorando postes)
    # ──────────────────────────────────────────────────────────────────────────
    def _sector_avg(self, ranges, ang_deg, vent_deg, max_d=10.0):
        """
        Devuelve la MEDIA aritmética de las lecturas válidas en el sector
        [ang_deg ± vent_deg]. Ignora postes y lecturas fuera de rango.
        Si no hay lecturas válidas devuelve max_d.
        """
        idx_c, idx_v = self._ang_to_idx(ang_deg, vent_deg)
        vals = []
        for i in range(idx_c - idx_v, idx_c + idx_v + 1):
            idx = i % self._n
            if self._post_mask is not None and self._post_mask[idx]:
                continue
            d = ranges[idx]
            if not (math.isnan(d) or math.isinf(d)) and 0.02 < d < max_d:
                vals.append(d)
        return (sum(vals) / len(vals)) if vals else max_d

    # ──────────────────────────────────────────────────────────────────────────
    #  MÍNIMO de sector angular (ignorando postes) — solo para zona trasera
    # ──────────────────────────────────────────────────────────────────────────
    def _sector_min(self, ranges, ang_deg, vent_deg, max_d=2.0):
        idx_c, idx_v = self._ang_to_idx(ang_deg, vent_deg)
        vals = []
        for i in range(idx_c - idx_v, idx_c + idx_v + 1):
            idx = i % self._n
            if self._post_mask is not None and self._post_mask[idx]:
                continue
            d = ranges[idx]
            if not (math.isnan(d) or math.isinf(d)) and 0.02 < d < max_d:
                vals.append(d)
        return min(vals) if vals else max_d

    # ──────────────────────────────────────────────────────────────────────────
    #  Conversión ángulo → índices de rayo
    # ──────────────────────────────────────────────────────────────────────────
    def _ang_to_idx(self, ang_deg, vent_deg):
        """
        Devuelve (idx_centro, idx_ventana) para un sector angular.
        Usa angle_min y angle_increment reales del LaserScan (no índices fijos).
        """
        ang_rad  = math.radians(ang_deg)
        vent_rad = math.radians(vent_deg)

        # Normalizar al rango del LIDAR
        ang_norm = ang_rad
        lim_max  = self._ang_min + 2 * math.pi
        while ang_norm < self._ang_min: ang_norm += 2 * math.pi
        while ang_norm > lim_max:       ang_norm -= 2 * math.pi

        idx_c = round((ang_norm  - self._ang_min) / self._ang_inc)
        idx_v = round(vent_rad   / self._ang_inc)
        return idx_c, idx_v

    # ──────────────────────────────────────────────────────────────────────────
    #  Delta de distancia lateral
    # ──────────────────────────────────────────────────────────────────────────
    def _delta_der(self):
        """
        Calcula el cambio de distancia promedio lateral comparando
        la mitad más reciente del buffer con la mitad más antigua.

        Retorna: (delta, hay_suficientes_muestras)
          delta > 0 → la distancia SUBIÓ  (hueco se abrió)
          delta < 0 → la distancia BAJÓ   (apareció un objeto)
        """
        buf = list(self._der_buf)
        if len(buf) < P.DELTA_N:
            return 0.0, False

        mitad = P.DELTA_N // 2
        vieja  = sum(buf[:mitad])  / mitad
        nueva  = sum(buf[mitad:])  / (len(buf) - mitad)
        return nueva - vieja, True

    # ──────────────────────────────────────────────────────────────────────────
    #  Utilidades generales
    # ──────────────────────────────────────────────────────────────────────────
    def _dist(self, x0=None, y0=None):
        if x0 is None:
            x0, y0 = self.ref_x, self.ref_y
        return math.hypot(self.x - x0, self.y - y0)

    def _set_ref(self):
        self.ref_x, self.ref_y = self.x, self.y

    def _err_yaw_abs(self):
        d = math.atan2(math.sin(self.yaw - self.yaw_calle),
                       math.cos(self.yaw - self.yaw_calle))
        return abs(math.degrees(d))

    def _err_yaw_signed(self):
        return math.degrees(
            math.atan2(math.sin(self.yaw - self.yaw_calle),
                       math.cos(self.yaw - self.yaw_calle))
        )

    def _pub(self, dir_dc, speed, servo, stop=0):
        m             = MotorCommand()
        m.dir_dc      = int(dir_dc)
        m.speed_dc    = int(speed)
        m.dir_servo   = int(servo)
        m.stop_lights = int(stop)
        self._dir     = int(dir_dc)
        self.pub.publish(m)

    def _pausa_ok(self):
        """
        Gestiona el temporizador de pausa de P.PAUSA_SEG segundos.
        Retorna True una sola vez cuando ha expirado.
        """
        now = self.get_clock().now()
        if self._t_pausa is None:
            self._t_pausa = now
            return False
        if (now - self._t_pausa).nanoseconds / 1e9 >= P.PAUSA_SEG:
            self._t_pausa = None
            return True
        return False

    def _reset_buf(self):
        """Vacía el buffer de delta para no arrastrar lecturas del estado anterior."""
        self._der_buf.clear()

    # ──────────────────────────────────────────────────────────────────────────
    #  Bucle de control (20 Hz)
    # ──────────────────────────────────────────────────────────────────────────
    def _loop(self):
        if not self._scan_ok:
            return

        delta, buf_lleno = self._delta_der()
        ey = self._err_yaw_abs()

        self.get_logger().info(
            f"[{self.estado:28s}] "
            f"avg_der={self.avg_der:.2f}m  "
            f"delta={delta:+.3f}m  "
            f"atras={self.dist_atras:.2f}m  "
            f"err={ey:.1f}°  "
            f"pos=({self.x:.3f},{self.y:.3f})",
            throttle_duration_sec=0.25
        )

        # ════════════════════════════════════════════════════════════════════
        #  ESPERANDO_LIDAR / CALIBRANDO — quieto
        # ════════════════════════════════════════════════════════════════════
        if self.estado in ('ESPERANDO_LIDAR', 'CALIBRANDO'):
            self._pub(0, 0, P.SERVO_CENTRO)

        # ════════════════════════════════════════════════════════════════════
        #  BUSCANDO_CARRO
        #  Avanza a velocidad normal. Detecta la 1ª caja por CAÍDA de delta:
        #  la distancia promedio derecha baja abruptamente al aparecer una caja.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'BUSCANDO_CARRO':
            self._pub(1, P.PWM_NORMAL, P.SERVO_CENTRO)

            if buf_lleno and delta < -P.DELTA_CERRAR:
                self._reset_buf()
                self.estado = 'SIGUIENDO_CARRO'
                self.get_logger().info(
                    f"1ª caja detectada (delta={delta:+.3f} m). "
                    "Reduciendo velocidad para buscar apertura del hueco..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  SIGUIENDO_CARRO
        #  Avanza lento junto a la 1ª caja. Detecta la apertura del hueco
        #  por SUBIDA de delta: la distancia sube cuando la caja se acaba.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'SIGUIENDO_CARRO':
            self._pub(1, P.PWM_LENTO, P.SERVO_CENTRO)

            if buf_lleno and delta > P.DELTA_ABRIR:
                self._reset_buf()
                self.estado = 'PARADO_INICIO_HUECO'
                self.get_logger().info(
                    f"Apertura detectada (delta={delta:+.3f} m). "
                    "Frenando para confirmar..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  PARADO_INICIO_HUECO
        #  Detención breve para estabilizar el buffer de delta.
        #  Si tras la pausa el delta sigue siendo positivo (hueco real),
        #  se marca el inicio del hueco y se comienza a medir.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'PARADO_INICIO_HUECO':
            self._pub(0, 0, P.SERVO_CENTRO)

            if self._pausa_ok():
                self._reset_buf()

                # Tomar avg_der actual para verificar que el hueco está abierto.
                # Como el buffer se vació, usamos directamente avg_der.
                if self.avg_der > (P.DELTA_ABRIR * 0.5 + 0.20):
                    # Promedio actual alto → hueco real
                    self.p1_x, self.p1_y = self.x, self.y
                    self.estado = 'MIDIENDO_HUECO'
                    self.get_logger().info(
                        f"Inicio confirmado (avg_der={self.avg_der:.2f} m). "
                        f"Midiendo a PWM={P.PWM_CREEP}..."
                    )
                else:
                    # Hueco se cerró durante la pausa → falso positivo
                    self.estado = 'SIGUIENDO_CARRO'
                    self.get_logger().warn(
                        f"Falso positivo (avg_der={self.avg_der:.2f} m). "
                        "Continuando..."
                    )

        # ════════════════════════════════════════════════════════════════════
        #  MIDIENDO_HUECO
        #  Avanza a velocidad creep mientras el hueco está abierto.
        #  Detecta la 2ª caja por CAÍDA de delta.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'MIDIENDO_HUECO':
            self._pub(1, P.PWM_CREEP, P.SERVO_CENTRO)
            avance = self._dist(self.p1_x, self.p1_y)

            if buf_lleno and delta < -P.DELTA_CERRAR:
                # 2ª caja detectada por caída abrupta de distancia
                self._hueco_long = avance
                self._reset_buf()
                self.estado = 'PARADO_FIN_HUECO'
                self.get_logger().info(
                    f"2ª caja detectada (delta={delta:+.3f} m). "
                    f"Longitud medida: {avance*100:.1f} cm. Frenando..."
                )

            elif avance > P.MAX_BUSQUEDA:
                self.get_logger().warn(
                    f"{avance*100:.0f} cm sin 2ª caja. Reiniciando búsqueda."
                )
                self._reset_buf()
                self.estado = 'BUSCANDO_CARRO'

        # ════════════════════════════════════════════════════════════════════
        #  PARADO_FIN_HUECO
        #  Detención breve al borde de la 2ª caja.
        #  Valida que el hueco tenga longitud suficiente.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'PARADO_FIN_HUECO':
            self._pub(0, 0, P.SERVO_CENTRO)

            if self._pausa_ok():
                if self._hueco_long >= P.MIN_HUECO:
                    self.yaw_calle = self.yaw
                    self._set_ref()
                    self.estado = 'AVANCE_POSICIONAMIENTO'
                    self.get_logger().info(
                        f"Hueco válido: {self._hueco_long*100:.1f} cm ≥ "
                        f"{P.MIN_HUECO*100:.0f} cm mínimo. "
                        f"Avanzando {P.AVANCE_POS*100:.0f} cm..."
                    )
                else:
                    self.get_logger().warn(
                        f"Hueco corto: {self._hueco_long*100:.1f} cm < "
                        f"{P.MIN_HUECO*100:.0f} cm. Buscando siguiente..."
                    )
                    self._reset_buf()
                    self.estado = 'SIGUIENDO_CARRO'

        # ════════════════════════════════════════════════════════════════════
        #  AVANCE_POSICIONAMIENTO
        #  Avanza hasta que la cola del carro supere el borde de la 2ª caja.
        #  AVANCE_POS = LIDAR_REAR (22 cm) + margen de giro (15 cm) = 37 cm
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'AVANCE_POSICIONAMIENTO':
            self._pub(1, P.PWM_NORMAL, P.SERVO_CENTRO)
            if self._dist() >= P.AVANCE_POS:
                self._set_ref()
                self.estado = 'REVERSA_GIRANDO'
                self.get_logger().info(
                    f"{P.AVANCE_POS*100:.0f} cm avanzados. "
                    "Iniciando reversa con giro a la derecha..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  REVERSA_GIRANDO
        #  Reversa + máximo giro derecha hasta alcanzar ~90° (por odometría).
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'REVERSA_GIRANDO':
            self._pub(2, P.PWM_NORMAL, P.SERVO_DER_REV)

            if self.dist_atras < P.PELIGRO_TRAS:
                self._set_ref()
                self.estado = 'RESCATE_ADELANTE'
                self.get_logger().warn(
                    f"Obstáculo trasero a {self.dist_atras:.2f} m. Rescatando..."
                )
            elif ey >= P.YAW_OBJ:
                self.estado = 'REVERSA_RECTA'
                self.get_logger().info(
                    f"{ey:.0f}° alcanzados. Entrando recto al hueco..."
                )

        # ════════════════════════════════════════════════════════════════════
        #  RESCATE_ADELANTE
        #  Avanza exactamente 20 cm (closed-loop, solo condición de distancia)
        #  para ganar espacio y reintentar el giro.
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'RESCATE_ADELANTE':
            self._pub(1, P.PWM_LENTO, P.SERVO_CENTRO)
            if self._dist() >= 0.20:
                self.estado = 'REVERSA_GIRANDO'
                self.get_logger().info("Rescate completado. Retomando giro...")

        # ════════════════════════════════════════════════════════════════════
        #  REVERSA_RECTA
        #  Entra recto con corrección proporcional de yaw.
        #  Velocidad lenta para lectura precisa del LIDAR.
        #
        #  DIST_STOP = 0.25 m:
        #    LIDAR centrado (22 cm de la cola) + bumper trasero a 3 cm de pared
        #    → LIDAR lee: espacio(50) - LIDAR_FRONT(22) - margen(3) = 25 cm
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'REVERSA_RECTA':
            eys   = self._err_yaw_signed()
            corr  = int(max(-P.MAX_CORR, min(P.MAX_CORR, P.KP_YAW * eys)))
            servo = P.SERVO_CENTRO - corr
            self._pub(2, P.PWM_LENTO, servo)

            if self.dist_atras < P.DIST_STOP:
                self.estado = 'ESTACIONADO'
                self.get_logger().info(
                    f"ESTACIONADO"
                    f"dist_atras={self.dist_atras*100:.0f} cm"
                    f"bumper trasero ≈ {(self.dist_atras - P.LIDAR_REAR)*100:.0f} cm de la pared"
                )

        # ════════════════════════════════════════════════════════════════════
        #  ESTACIONADO
        # ════════════════════════════════════════════════════════════════════
        elif self.estado == 'ESTACIONADO':
            self._pub(0, 0, P.SERVO_CENTRO, stop=1)


def main():
    rclpy.init()
    rclpy.spin(SmartParking())
    rclpy.shutdown()


if __name__ == '__main__':
    main()