#!/usr/bin/env python3
import rclpy
from driving.vision_utils import segmentacion_color
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection')

        # --- PARÁMETROS DE SEGMENTACIÓN ---
        self.declare_parameter('umbral_blanco', 110)
        self.declare_parameter('rojo_y_sup', 380)
        self.declare_parameter('rojo_y_inf', 470)
        self.declare_parameter('rojo_ancho_sup', 500)
        self.declare_parameter('rojo_ancho_inf', 600)

        # --- PARÁMETROS DE ROBUSTEZ ---
        self.declare_parameter('min_puntos', 50)
        self.declare_parameter('base_offset_l', 180) 
        self.declare_parameter('base_offset_r', 180)
        self.declare_parameter('usar_anclaje', True)

        self.left_fit = None
        self.center_fit = None
        self.right_fit = None
        
        self.bridge = CvBridge()
        self.lane_width_pixels = 350.0  # Ancho estimado inicial
        
        # Filtros de suavizado (Alpha Filter)
        self.left_fit = None
        self.right_fit = None

        # --- SUBSCRIPCIÓN ---
        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            10)

        # --- PUBLICADORES ---
        self.pub_mask = self.create_publisher(CompressedImage, '/vision/lane_detection_mask/compressed', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/vision/lane_detection_debug/compressed', 10)
        self.pub_info = self.create_publisher(String, '/vision/lane_info', 10)

    def fit_lane_quadratic(self, mask, line_type, cx, y_inf):
        """
        line_type: 'L' (Izquierda), 'C' (Centro), 'R' (Derecha)
        """
        ys, xs = np.where(mask > 0)
        h, w = mask.shape[:2]
        
        # --- NUEVA LÓGICA DE SEGMENTACIÓN POR TERCIOS ---
        # Dividimos el ancho en zonas para separar las 3 líneas
        if line_type == 'L':
            mask_indices = xs < (w * 0.33)
        elif line_type == 'C':
            mask_indices = (xs >= (w * 0.33)) & (xs < (w * 0.66))
        else: # 'R'
            mask_indices = xs >= (w * 0.66)

        puntos = np.column_stack((xs[mask_indices], ys[mask_indices]))

        if len(puntos) < self.get_parameter('min_puntos').value:
            return None

        px, py = puntos[:, 0], puntos[:, 1]

        # Anclajes específicos para las 3 líneas
        if self.get_parameter('usar_anclaje').value:
            if line_type == 'L': anchor_x = cx - 250 # Ajusta según tu pista
            elif line_type == 'C': anchor_x = cx
            else: anchor_x = cx + 250
            
            px = np.concatenate((px, np.full(40, anchor_x)))
            py = np.concatenate((py, np.full(40, y_inf)))

        try:
            return np.polyfit(py, px, 2)
        except:
            return None

    def draw_two_lanes(self, img, l_fit, c_fit, r_fit, y_s, y_i):
        overlay = img.copy()
        y_range = np.linspace(y_s, y_i, 20)
        
        def get_px(fit):
            return fit[0]*y_range**2 + fit[1]*y_range + fit[2]

        # --- CARRIL IZQUIERDO (L a C) ---
        if l_fit is not None and c_fit is not None:
            l_px, c_px = get_px(l_fit), get_px(c_fit)
            pts_l = np.column_stack((l_px, y_range)).astype(np.int32)
            pts_c = np.flipud(np.column_stack((c_px, y_range)).astype(np.int32))
            cv2.fillPoly(overlay, [np.vstack((pts_l, pts_c))], (0, 255, 0)) # Verde

        # --- CARRIL DERECHO (C a R) ---
        if c_fit is not None and r_fit is not None:
            c_px, r_px = get_px(c_fit), get_px(r_fit)
            pts_c = np.column_stack((c_px, y_range)).astype(np.int32)
            pts_r = np.flipud(np.column_stack((r_px, y_range)).astype(np.int32))
            cv2.fillPoly(overlay, [np.vstack((pts_c, pts_r))], (255, 255, 0)) # Amarillo (Cian)

        return cv2.addWeighted(overlay, 0.3, img, 0.7, 0)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error CVBridge: {e}")
            return

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        cx = w // 2

        # --- 1. CONFIGURAR ROI ---
        y_s = self.get_parameter('rojo_y_sup').value
        y_i = self.get_parameter('rojo_y_inf').value
        w_s = self.get_parameter('rojo_ancho_sup').value
        w_i = self.get_parameter('rojo_ancho_inf').value
        
        poly = np.array([[(cx - w_i//2, y_i), (cx + w_i//2, y_i), 
                          (cx + w_s//2, y_s), (cx - w_s//2, y_s)]], np.int32)

        # --- 2. SEGMENTACIÓN ---
        zona = segmentacion_color(frame, poly, self.get_parameter('umbral_blanco').value)

        # --- 3. PROCESAMIENTO CUADRÁTICO (Las 3 líneas) ---
        fL_raw = self.fit_lane_quadratic(zona, 'L', cx, y_i)
        fC_raw = self.fit_lane_quadratic(zona, 'C', cx, y_i)
        fR_raw = self.fit_lane_quadratic(zona, 'R', cx, y_i)

        # --- 4. SUAVIZADO (Alpha Filter) ---
        # Asegúrate de tener self.fit_L, self.fit_C y self.fit_R inicializados en el __init__
        alpha = 0.4
        if fL_raw is not None:
            self.left_fit = fL_raw if self.left_fit is None else alpha*fL_raw + (1-alpha)*self.left_fit
        if fC_raw is not None:
            self.center_fit = fC_raw if self.center_fit is None else alpha*fC_raw + (1-alpha)*self.center_fit
        if fR_raw is not None:
            self.right_fit = fR_raw if self.right_fit is None else alpha*fR_raw + (1-alpha)*self.right_fit

        # --- 5. DETERMINAR ESTADO Y LÓGICA ---
        lineas_vistas = [f for f in [fL_raw, fC_raw, fR_raw] if f is not None]
        num_lineas = len(lineas_vistas)
        
        if num_lineas >= 2:
            estado = f"MULTI_LANE ({num_lineas} lines)"
        elif num_lineas == 1:
            estado = "SINGLE_LINE"
        else:
            estado = "NO_LANES"

        # --- 6. VISUALIZACIÓN ---
        debug = frame.copy()
        # Usamos los fits suavizados para el dibujo
        debug = self.draw_two_lanes(debug, self.left_fit, self.center_fit, self.right_fit, y_s, y_i)

        # Nube de puntos para debug
        ys, xs = np.where(zona > 0)
        if len(xs) > 0:
            for i in range(0, len(xs), 40): # Menos puntos para no alentar el proceso
                cv2.circle(debug, (xs[i], ys[i] + y_s), 2, (255, 0, 0), -1)

        cv2.polylines(debug, poly, True, (0, 0, 255), 2)
        cv2.putText(debug, f"Estado: {estado}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # --- 7. PUBLICACIÓN ---
        self.pub_info.publish(String(data=estado))
        self.pub_mask.publish(self.bridge.cv2_to_compressed_imgmsg(zona))
        self.pub_debug.publish(self.bridge.cv2_to_compressed_imgmsg(debug))

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
