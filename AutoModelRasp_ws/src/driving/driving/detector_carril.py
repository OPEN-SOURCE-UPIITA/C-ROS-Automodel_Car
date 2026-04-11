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

        # --- 1. PARÁMETROS DINÁMICOS (RQT) ---
        self.declare_parameter('umbral_blanco', 170)
        self.declare_parameter('corte_y_sup_pct', 40)
        self.declare_parameter('corte_y_inf_pct', 10)
        self.declare_parameter('ancho_top', 120)
        self.declare_parameter('ancho_bot', 350) 
        self.declare_parameter('suavizado_pct', 30)
        self.declare_parameter('min_puntos', 50)
        self.declare_parameter('min_pendiente', 0.8) 
        self.declare_parameter('base_offset_l', 175) 
        self.declare_parameter('base_offset_r', 175)
        self.declare_parameter('activar_seguimiento', True)
        self.declare_parameter('confianza_un_carril', 0.6)
        self.declare_parameter('usar_anclaje_base', True)
        self.declare_parameter('max_curvatura', 0.0015)

        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- 2. VARIABLES DE ESTADO ---
        self.bridge = CvBridge()
        self.last_error = 0.0
        self.left_fit = None
        self.right_fit = None
        self.lane_width_pixels = 350.0 
        self.carril_actual = "Desconocido"

        # --- 3. COMUNICACIONES ---
        self.subscription = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.image_callback, 10)
        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def segmentacion_color(self, frame):
        """Genera la máscara de blancos para toda la imagen."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        umbral = self.get_parameter('umbral_blanco').value
        _, mask_blancos = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)
        return mask_blancos

    def which_carril(self, mask, debug_img):
        """Detecta la posición del auto basándose en un ROI panorámico inferior."""
        h, w = mask.shape
        # Polígono Rojo: Panorámico inferior para ver todas las líneas de la pista
        roi_rojo_pts = np.array([[(50, h-10), (w-50, h-10), (w-150, h-100), (150, h-100)]], np.int32)
        cv2.polylines(debug_img, roi_rojo_pts, True, (0, 0, 255), 3)

        # Lógica de histograma para detectar líneas en el carril
        mask_roi_rojo = np.zeros_like(mask)
        cv2.fillPoly(mask_roi_rojo, roi_rojo_pts, 255)
        zona_interes = cv2.bitwise_and(mask, mask_roi_rojo)
        
        # Opcional: Aquí podrías implementar el conteo de picos del histograma
        # histogram = np.sum(zona_interes[h-100:h-10, :], axis=0)
        
        msg = "CARRIL DETECTADO" # Placeholder para tu lógica de conteo
        cv2.putText(debug_img, f"POS: {msg}", (20, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return msg

    def fit_lane_logic(self, points, anchor_x, h):
        min_pts = self.get_parameter('min_puntos').value
        if len(points) < min_pts: return None
        
        xs, ys = points[:, 0], points[:, 1]
        if self.get_parameter('usar_anclaje_base').value:
            num_anchors = 25
            xs = np.concatenate((xs, np.full(num_anchors, anchor_x)))
            ys = np.concatenate((ys, np.full(num_anchors, h)))
        
        try:
            coeffs = np.polyfit(ys, xs, 2)
            max_curv = self.get_parameter('max_curvatura').value
            if abs(coeffs[0]) > max_curv:
                coeffs_lin = np.polyfit(ys, xs, 1)
                coeffs = np.array([0.0, coeffs_lin[0], coeffs_lin[1]])

            min_slope = self.get_parameter('min_pendiente').value
            if abs(2 * coeffs[0] * h + coeffs[1]) > (1.0 / min_slope): return None
            return coeffs
        except: return None

    def render_poly_restricted(self, img, coeffs, color, y_min, y_max, side):
        cx_img = img.shape[1] // 2
        plot_y = np.linspace(y_min, y_max, 25)
        plot_x = coeffs[0]*plot_y**2 + coeffs[1]*plot_y + coeffs[2]
        
        if side == 'L': plot_x = np.minimum(plot_x, cx_img - 2)
        else: plot_x = np.maximum(plot_x, cx_img + 2)
        
        pts = np.array([np.transpose(np.vstack([plot_x, plot_y]))], np.int32)
        cv2.polylines(img, pts, False, color, 4)

    def dibujo_virtual(self, img, polygon, l_fit, r_fit, target_x, anchors, y_lims):
        y_sup, y_inf = y_lims
        cx_img = img.shape[1] // 2
        cv2.polylines(img, polygon, True, (255, 0, 0), 2)
        cv2.line(img, (cx_img, y_inf), (cx_img, y_sup), (255, 255, 0), 1)
        if self.get_parameter('usar_anclaje_base').value:
            cv2.circle(img, (anchors[0], y_inf), 7, (255, 0, 255), -1)
            cv2.circle(img, (anchors[1], y_inf), 7, (255, 0, 255), -1)
        if l_fit is not None: self.render_poly_restricted(img, l_fit, (0, 255, 0), y_sup, y_inf, 'L')
        if r_fit is not None: self.render_poly_restricted(img, r_fit, (0, 255, 0), y_sup, y_inf, 'R')
        if target_x is not None:
            cv2.circle(img, (int(target_x), y_sup), 12, (0, 255, 255), -1)
            cv2.line(img, (cx_img, y_sup), (int(target_x), y_sup), (0, 255, 255), 2)

    def image_callback(self, msg):
        try: frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        cx = w // 2
        debug = frame.copy()

        y_sup = int(h * (self.get_parameter('corte_y_sup_pct').value / 100.0))
        y_inf = h - int(h * (self.get_parameter('corte_y_inf_pct').value / 100.0))
        w_t, w_b = self.get_parameter('ancho_top').value, self.get_parameter('ancho_bot').value
        polygon = np.array([[(cx - w_b//2, y_inf), (cx + w_b//2, y_inf), 
                             (cx + w_t, y_sup), (cx - w_t, y_sup)]], np.int32)

        # 1. Segmentación Base (Toda la imagen)
        mask_completa = self.segmentacion_color(frame)

        # 2. Localización (Cuadro Rojo)
        self.carril_actual = self.which_carril(mask_completa, debug)

        # 3. Máscara de Control (Cuadro Azul)
        mask_roi_azul = np.zeros_like(mask_completa)
        cv2.fillPoly(mask_roi_azul, polygon, 255)
        mask_control = cv2.bitwise_and(mask_completa, mask_roi_azul)

        # 4. Procesamiento de Puntos
        off_l, off_r = self.get_parameter('base_offset_l').value, self.get_parameter('base_offset_r').value
        anchors = (cx - off_l, cx + off_r)
        
        ys, xs = np.where(mask_control > 0)
        l_fit_raw, r_fit_raw = None, None
        if len(xs) > 0:
            pts = np.column_stack((xs, ys))
            l_fit_raw = self.fit_lane_logic(pts[pts[:, 0] < cx], anchors[0], y_inf)
            r_fit_raw = self.fit_lane_logic(pts[pts[:, 0] > cx], anchors[1], y_inf)

        # 5. Suavizado Temporal
        alpha = self.get_parameter('suavizado_pct').value / 100.0
        if l_fit_raw is not None:
            if self.left_fit is None: self.left_fit = l_fit_raw
            else:
                diff = np.abs(l_fit_raw[1] - self.left_fit[1])
                a = alpha if diff < 0.5 else 0.8
                self.left_fit = a * l_fit_raw + (1 - a) * self.left_fit
        else: self.left_fit = None

        if r_fit_raw is not None:
            if self.right_fit is None: self.right_fit = r_fit_raw
            else:
                diff = np.abs(r_fit_raw[1] - self.right_fit[1])
                a = alpha if diff < 0.5 else 0.8
                self.right_fit = a * r_fit_raw + (1 - a) * self.right_fit
        else: self.right_fit = None

        # 6. Cálculo de Target X
        tx_l, tx_r = None, None
        if self.left_fit is not None:
            tx_l = np.polyval(self.left_fit, y_sup)
            tx_l = min(tx_l, cx - 10)
        if self.right_fit is not None:
            tx_r = np.polyval(self.right_fit, y_sup)
            tx_r = max(tx_r, cx + 10)

        target_x = cx
        confianza = 1.0
        if tx_l is not None and tx_r is not None:
            target_x = (tx_l + tx_r) // 2
            self.lane_width_pixels = 0.95 * self.lane_width_pixels + 0.05 * (tx_r - tx_l)
        elif tx_l is not None:
            target_x = tx_l + (self.lane_width_pixels / 2)
            confianza = self.get_parameter('confianza_un_carril').value
        elif tx_r is not None:
            target_x = tx_r - (self.lane_width_pixels / 2)
            confianza = self.get_parameter('confianza_un_carril').value

        # 7. Publicación de Error
        error_final = np.clip(((target_x - cx) / float(cx)) * confianza, -1.0, 1.0)
        self.last_error = 0.7 * self.last_error + 0.3 * error_final
        if self.get_parameter('activar_seguimiento').value:
            self.publisher_error.publish(Float32(data=float(self.last_error)))

        # 8. Visualización y Publicación
        self.dibujo_virtual(debug, polygon, self.left_fit, self.right_fit, target_x, anchors, (y_sup, y_inf))
        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(mask_control))
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
