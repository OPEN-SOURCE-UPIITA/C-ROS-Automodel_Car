#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

# ==============================================================================
# DETECTOR DE CRUCES PEATONALES (Filtro HLS + ROI Trapezoidal)
# ==============================================================================

class DetectorCrucesNode(Node):
    def __init__(self):
        super().__init__('detector_cruces')

        # =============================
        # PARÁMETROS DINÁMICOS (RQT)
        # =============================
        self.declare_parameter('resize_w', 320)
        self.declare_parameter('resize_h', 240)

        self.declare_parameter('roi_y_ratio', 0.5)
        self.declare_parameter('roi_ancho_sup', 120)
        self.declare_parameter('roi_ancho_inf', 300)

        # Filtros de Color HLS (Luminosidad y Saturación)
        self.declare_parameter('l_min', 200)
        self.declare_parameter('s_max', 60)

        # Lógica de detección del cruce
        self.declare_parameter('ancho_min_linea', 60)
        self.declare_parameter('num_lineas_cruce', 3)
        self.declare_parameter('salto_filas', 2)

        self.add_on_set_parameters_callback(self.parameters_callback)

        # =============================
        # COMUNICACIÓN
        # =============================
        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            10
        )

        self.pub_distancia = self.create_publisher(
            Float32,
            '/vision/cruce_peatonal/distancia',
            10
        )

        self.pub_debug = self.create_publisher(
            CompressedImage,
            '/vision/cruce_peatonal/debug/compressed',
            10
        )

        # NUEVO PUBLICADOR: Máscara HLS para verla en RQT
        self.pub_mask = self.create_publisher(
            CompressedImage,
            '/vision/cruce_peatonal/mask/compressed',
            10
        )

        self.bridge = CvBridge()

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # =============================
        # LEER PARAMS
        # =============================
        w_r = self.get_parameter('resize_w').value
        h_r = self.get_parameter('resize_h').value
        roi_ratio = self.get_parameter('roi_y_ratio').value

        l_min = self.get_parameter('l_min').value
        s_max = self.get_parameter('s_max').value

        ancho_min = self.get_parameter('ancho_min_linea').value
        lineas_req = self.get_parameter('num_lineas_cruce').value
        salto = self.get_parameter('salto_filas').value

        w_top = self.get_parameter('roi_ancho_sup').value
        w_bot = self.get_parameter('roi_ancho_inf').value

        # =============================
        # PREPROCESAMIENTO
        # =============================
        frame = cv2.resize(frame, (w_r, h_r))
        h, w = frame.shape[:2]
        cx = w // 2

        y1 = int(h * roi_ratio)
        y2 = h

        # ROI trapezoidal
        polygon = np.array([[
            (cx - w_bot//2, y2),
            (cx + w_bot//2, y2),
            (cx + w_top//2, y1),
            (cx - w_top//2, y1)
        ]], np.int32)

        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)

        roi = frame[y1:y2, :]

        # =============================
        # FILTRO HLS (Blanco)
        # =============================
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        _, L, S = cv2.split(hls)

        mask = (L > l_min) & (S < s_max)
        mask = mask.astype(np.uint8) * 255

        # aplicar ROI
        mask = cv2.bitwise_and(mask, mask_roi[y1:y2, :])

        # limpieza morfológica
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # =============================
        # HISTOGRAMA Y DETECCIÓN
        # =============================
        hist = np.count_nonzero(mask, axis=1)

        lineas = 0
        en_linea = False
        y_detect = 0

        for y in range(len(hist) - 1, 0, -salto):
            if hist[y] > ancho_min:
                if not en_linea:
                    lineas += 1
                    en_linea = True

                    # Guardamos la posición Y local de la primera línea detectada
                    if lineas == 1:
                        y_detect = y

                if lineas >= lineas_req:
                    break
            else:
                en_linea = False

        # =============================
        # RESULTADO Y PUBLICACIÓN
        # =============================
        distancia = 999.0
        debug = frame.copy()

        # dibujar ROI azul
        cv2.polylines(debug, polygon, True, (255, 0, 0), 2)

        if lineas >= lineas_req:
            # Calculamos la coordenada Y global (relativa a toda la imagen, no solo al ROI)
            y_global = y1 + y_detect
            
            # Enviamos el píxel exacto al multiplexor en lugar de un valor normalizado
            distancia = float(y_global)

            cv2.putText(debug, f"CRUCE {lineas}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

            cv2.line(debug, (0, y_global), (w, y_global), (0, 0, 255), 2)

        # Convertir máscara de 1 canal a 3 canales BGR para que RQT no de errores de formato
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        self.pub_distancia.publish(Float32(data=distancia))
        self.pub_debug.publish(self.bridge.cv2_to_compressed_imgmsg(debug))
        self.pub_mask.publish(self.bridge.cv2_to_compressed_imgmsg(mask_bgr))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorCrucesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
