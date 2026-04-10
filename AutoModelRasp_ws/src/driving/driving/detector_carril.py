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

        # --- 1. DECLARACIÓN DE PARÁMETROS DINÁMICOS (Configurables desde RQT) ---
        self.declare_parameter('umbral_blanco', 170)
        self.declare_parameter('corte_y_pct', 40)
        self.declare_parameter('ancho_top', 120)
        self.declare_parameter('ancho_bot', 350) 
        self.declare_parameter('suavizado_pct', 30)
        self.declare_parameter('min_puntos', 50)
        self.declare_parameter('min_pendiente', 0.8) 
        self.declare_parameter('activar_seguimiento', True)
        
        # NUEVOS: Puntos de anclaje base (distancia desde el centro hacia los lados)
        self.declare_parameter('base_offset_l', 175) # Posición X inicial del carril izq en la base
        self.declare_parameter('base_offset_r', 175) # Posición X inicial del carril der en la base

        # --- 2. REGISTRO DEL CALLBACK PARA RQT ---
        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- 3. COMUNICACIONES ROS ---
        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            10)

        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)

        self.bridge = CvBridge()
        self.last_error = 0.0
        self.left_fit = None
        self.right_fit = None
        self.lane_width_pixels = 350.0 

    def parameters_callback(self, params):
        """Callback para actualizar parámetros desde RQT en tiempo real."""
        for param in params:
            self.get_logger().info(f"RQT Actualizado: {param.name} = {param.value}")
        return SetParametersResult(successful=True)

    def fit_lane_anchored(self, points, anchor_x, h):
        """Ajusta una parábola forzando el paso por un punto de anclaje en la base."""
        min_pts = self.get_parameter('min_puntos').value
        min_slope = self.get_parameter('min_pendiente').value
        
        if len(points) < min_pts:
            return None
        
        xs = points[:, 0]
        ys = points[:, 1]
        
        # INYECCIÓN DE PUNTOS VIRTUALES (Anclaje)
        # Añadimos 25 puntos en la base para "pesar" la regresión hacia el ancla
        num_anchors = 25
        anchor_ys = np.full(num_anchors, h)
        anchor_xs = np.full(num_anchors, anchor_x)
        
        xs_ext = np.concatenate((xs, anchor_xs))
        ys_ext = np.concatenate((ys, anchor_ys))
        
        try:
            coeffs = np.polyfit(ys_ext, xs_ext, 2)
            # Filtro de horizontalidad en la base
            dx_dy = abs(2 * coeffs[0] * h + coeffs[1])
            if dx_dy > (1.0 / min_slope): 
                return None
            return coeffs
        except Exception:
            return None

    def draw_lane_poly(self, img, coeffs, color, y_min, y_max):
        """Dibuja la parábola y retorna el punto objetivo en el tope."""
        plot_y = np.linspace(y_min, y_max, 25)
        plot_x = coeffs[0]*plot_y**2 + coeffs[1]*plot_y + coeffs[2]
        pts = np.array([np.transpose(np.vstack([plot_x, plot_y]))], np.int32)
        cv2.polylines(img, pts, False, color, 5)
        
        target_x = coeffs[0]*(y_min**2) + coeffs[1]*y_min + coeffs[2]
        return int(target_x)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        frame = cv2.resize(frame, (640, 480))
        h, w, _ = frame.shape
        debug = frame.copy()

        # --- LEER PARÁMETROS DINÁMICOS ---
        umbral = self.get_parameter('umbral_blanco').value
        corte_y_val = self.get_parameter('corte_y_pct').value
        w_t = self.get_parameter('ancho_top').value
        w_b = self.get_parameter('ancho_bot').value
        alpha = self.get_parameter('suavizado_pct').value / 100.0
        
        # Parámetros de anclaje base
        offset_l = self.get_parameter('base_offset_l').value
        offset_r = self.get_parameter('base_offset_r').value
        
        corte_y = int(h * (corte_y_val / 100.0))
        cx = w // 2

        # Puntos de anclaje base (rosados para depuración)
        anchor_l_x = cx - offset_l
        anchor_r_x = cx + offset_r
        cv2.circle(debug, (anchor_l_x, h-5), 8, (255, 0, 255), -1)
        cv2.circle(debug, (anchor_r_x, h-5), 8, (255, 0, 255), -1)

        # --- SEGMENTACIÓN ---
        polygon = np.array([[(cx - w_b//2, h), (cx + w_b//2, h), 
                             (cx + w_t, corte_y), (cx - w_t, corte_y)]], np.int32)
        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask_blancos = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask_blancos, mask_roi)

        # --- PROCESAMIENTO ---
        ys, xs = np.where(mask > 0)
        target_x = cx 

        if len(xs) > 0:
            points = np.column_stack((xs, ys))
            left_pts = points[points[:, 0] < cx]
            right_pts = points[points[:, 0] >= cx]

            # Ajuste usando las anclas fijas en la base
            l_fit = self.fit_lane_anchored(left_pts, anchor_l_x, h)
            r_fit = self.fit_lane_anchored(right_pts, anchor_r_x, h)

            x_l, x_r = None, None

            if l_fit is not None:
                self.left_fit = l_fit if self.left_fit is None else alpha * l_fit + (1-alpha) * self.left_fit
                x_l = self.draw_lane_poly(debug, self.left_fit, (0, 255, 0), corte_y, h)
            
            if r_fit is not None:
                self.right_fit = r_fit if self.right_fit is None else alpha * r_fit + (1-alpha) * self.right_fit
                x_r = self.draw_lane_poly(debug, self.right_fit, (0, 255, 0), corte_y, h)

            # Lógica de Target con memoria de ancho
            if x_l is not None and x_r is not None:
                target_x = (x_l + x_r) // 2
                self.lane_width_pixels = 0.95 * self.lane_width_pixels + 0.05 * (x_r - x_l)
            elif x_l is not None:
                target_x = x_l + int(self.lane_width_pixels / 2)
            elif x_r is not None:
                target_x = x_r - int(self.lane_width_pixels / 2)

            # Publicar Error
            error = np.clip((target_x - cx) / float(cx), -1.0, 1.0)
            self.last_error = 0.7 * self.last_error + 0.3 * error
            
            if self.get_parameter('activar_seguimiento').value:
                self.publisher_error.publish(Float32(data=float(self.last_error)))

            # Visualización final
            cv2.polylines(debug, polygon, True, (255, 0, 0), 2)
            cv2.circle(debug, (int(target_x), corte_y), 12, (0, 255, 255), -1)

        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(mask))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(debug))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorCarrilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()