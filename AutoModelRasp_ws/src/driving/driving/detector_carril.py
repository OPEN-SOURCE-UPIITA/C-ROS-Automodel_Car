#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorCarrilNode(Node):
    def __init__(self):
        super().__init__('detector_carril')

        # --- 1. DECLARACIÓN DE PARÁMETROS DINÁMICOS (RQT) ---
        self.declare_parameter('umbral_blanco', 170)
        self.declare_parameter('corte_y_sup_pct', 40) # Horizonte
        self.declare_parameter('corte_y_inf_pct', 10) # Para ocultar el cofre (desde abajo)
        self.declare_parameter('ancho_top', 120)
        self.declare_parameter('ancho_bot', 350) 
        self.declare_parameter('suavizado_pct', 30)
        self.declare_parameter('min_puntos', 50)
        self.declare_parameter('min_pendiente', 0.8) 
        self.declare_parameter('base_offset_l', 175) 
        self.declare_parameter('base_offset_r', 175)
        self.declare_parameter('activar_seguimiento', True)

        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- 2. COMUNICACIONES ---
        self.subscription = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.image_callback, 10)
        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)

        self.bridge = CvBridge()
        self.last_error = 0.0
        self.left_fit = None
        self.right_fit = None
        self.lane_width_pixels = 350.0 

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    # --- MÓDULO: SEGMENTACIÓN ---
    def segmentacion_color(self, frame, polygon):
        """Aplica umbral de blancos y máscara de ROI."""
        h, w = frame.shape[:2]
        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        umbral = self.get_parameter('umbral_blanco').value
        _, mask_blancos = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)
        
        mask_final = cv2.bitwise_and(mask_blancos, mask_roi)
        return mask_final

    # --- MÓDULO: PROCESAMIENTO DE PUNTOS ---
    def procesar_puntos_carril(self, mask, cx, h, anchor_l, anchor_r):
        """Busca puntos y ajusta las parábolas con anclaje."""
        ys, xs = np.where(mask > 0)
        l_fit, r_fit = None, None
        
        if len(xs) > 0:
            points = np.column_stack((xs, ys))
            left_pts = points[points[:, 0] < cx]
            right_pts = points[points[:, 0] >= cx]

            l_fit = self.fit_lane_anchored(left_pts, anchor_l, h)
            r_fit = self.fit_lane_anchored(right_pts, anchor_r, h)
            
        return l_fit, r_fit

    def fit_lane_anchored(self, points, anchor_x, h):
        """Ajuste matemático con peso en la base."""
        min_pts = self.get_parameter('min_puntos').value
        if len(points) < min_pts: return None
        
        xs, ys = points[:, 0], points[:, 1]
        # Inyectar ancla base (y=h) para estabilidad
        num_anchors = 25
        xs_ext = np.concatenate((xs, np.full(num_anchors, anchor_x)))
        ys_ext = np.concatenate((ys, np.full(num_anchors, h)))
        
        try:
            coeffs = np.polyfit(ys_ext, xs_ext, 2)
            # Filtro de horizontalidad
            min_slope = self.get_parameter('min_pendiente').value
            if abs(2 * coeffs[0] * h + coeffs[1]) > (1.0 / min_slope): return None
            return coeffs
        except: return None

    # --- MÓDULO: DIBUJO VIRTUAL ---
    def dibujo_virtual(self, img, polygon, l_fit, r_fit, target_x, anchors, y_lims):
        """Dibuja toda la interfaz de depuración sobre la imagen."""
        y_sup, y_inf = y_lims
        # 1. Polígono ROI (Azul)
        cv2.polylines(img, polygon, True, (255, 0, 0), 2)
        
        # 2. Anclas base (Rosado)
        cv2.circle(img, (anchors[0], y_inf), 7, (255, 0, 255), -1)
        cv2.circle(img, (anchors[1], y_inf), 7, (255, 0, 255), -1)

        # 3. Líneas de interpolación (Verde)
        x_target_l, x_target_r = None, None
        if l_fit is not None:
            x_target_l = self.render_poly(img, l_fit, (0, 255, 0), y_sup, y_inf)
        if r_fit is not None:
            x_target_r = self.render_poly(img, r_fit, (0, 255, 0), y_sup, y_inf)

        # 4. Target Point (Cian)
        if target_x is not None:
            cv2.circle(img, (int(target_x), y_sup), 12, (0, 255, 255), -1)
            cv2.line(img, (img.shape[1]//2, y_sup), (int(target_x), y_sup), (0, 255, 255), 2)
        
        return x_target_l, x_target_r

    def render_poly(self, img, coeffs, color, y_min, y_max):
        plot_y = np.linspace(y_min, y_max, 25)
        plot_x = coeffs[0]*plot_y**2 + coeffs[1]*plot_y + coeffs[2]
        pts = np.array([np.transpose(np.vstack([plot_x, plot_y]))], np.int32)
        cv2.polylines(img, pts, False, color, 4)
        return coeffs[0]*(y_min**2) + coeffs[1]*y_min + coeffs[2]

    # --- CALLBACK PRINCIPAL ---
    def image_callback(self, msg):
        try: frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        cx = w // 2
        debug = frame.copy()

        # Obtener cortes Y
        y_sup = int(h * (self.get_parameter('corte_y_sup_pct').value / 100.0))
        y_inf = h - int(h * (self.get_parameter('corte_y_inf_pct').value / 100.0))

        # Definir polígono con corte inferior
        w_t, w_b = self.get_parameter('ancho_top').value, self.get_parameter('ancho_bot').value
        polygon = np.array([[(cx - w_b//2, y_inf), (cx + w_b//2, y_inf), 
                             (cx + w_t, y_sup), (cx - w_t, y_sup)]], np.int32)

        # 1. Segmentación
        mask = self.segmentacion_color(frame, polygon)

        # 2. Procesamiento de puntos
        off_l, off_r = self.get_parameter('base_offset_l').value, self.get_parameter('base_offset_r').value
        anchor_l, anchor_r = cx - off_l, cx + off_r
        
        l_fit_raw, r_fit_raw = self.procesar_puntos_carril(mask, cx, y_inf, anchor_l, anchor_r)

        # Suavizado temporal
        alpha = self.get_parameter('suavizado_pct').value / 100.0
        if l_fit_raw is not None:
            self.left_fit = l_fit_raw if self.left_fit is None else alpha * l_fit_raw + (1-alpha) * self.left_fit
        if r_fit_raw is not None:
            self.right_fit = r_fit_raw if self.right_fit is None else alpha * r_fit_raw + (1-alpha) * self.right_fit

        # 3. Cálculo de Target X con memoria
        tx_l, tx_r = None, None
        # Necesitamos calcular los puntos de dibujo primero para el target
        if self.left_fit is not None: tx_l = self.left_fit[0]*(y_sup**2) + self.left_fit[1]*y_sup + self.left_fit[2]
        if self.right_fit is not None: tx_r = self.right_fit[0]*(y_sup**2) + self.right_fit[1]*y_sup + self.right_fit[2]

        target_x = cx
        if tx_l is not None and tx_r is not None:
            target_x = (tx_l + tx_r) // 2
            self.lane_width_pixels = 0.95 * self.lane_width_pixels + 0.05 * (tx_r - tx_l)
        elif tx_l is not None: target_x = tx_l + (self.lane_width_pixels / 2)
        elif tx_r is not None: target_x = tx_r - (self.lane_width_pixels / 2)

        # 4. Dibujo Virtual
        self.dibujo_virtual(debug, polygon, self.left_fit, self.right_fit, target_x, (anchor_l, anchor_r), (y_sup, y_inf))

        # Publicar Error
        error = np.clip((target_x - cx) / float(cx), -1.0, 1.0)
        self.last_error = 0.7 * self.last_error + 0.3 * error
        if self.get_parameter('activar_seguimiento').value:
            self.publisher_error.publish(Float32(data=float(self.last_error)))

        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(mask))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(debug))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorCarrilNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()