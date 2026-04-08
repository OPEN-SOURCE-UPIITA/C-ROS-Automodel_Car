#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage  # 1. Importación añadida
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorCarrilNode(Node):
    def __init__(self):
        super().__init__('detector_carril')

        self.get_logger().info("Iniciando Detector de Carril (Comprimido + Parametros RQT)")

        # --- PARAMETROS DINAMICOS ---
        # Adios a todos los min/max de HSV. Hola a la simplicidad.
        self.declare_parameter('umbral_blanco', 180) # De 0 a 255
        # ... (los demas parametros de trapecio se quedan igual)
        self.declare_parameter('corte_y_pct', 55)
        self.declare_parameter('ancho_top', 120)
        self.declare_parameter('ancho_bot', 600)
        self.declare_parameter('suavizado_pct', 20)
        self.declare_parameter('ancho_carril_px', 350)

        # --- SUSCRIPCIONES Y PUBLICADORES ---
        self.subscription = self.create_subscription(
            Image, 
            '/ascamera_hp60c/camera_publisher/rgb0/image', 
            self.image_callback, 
            10)
        
        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        
        # 2. Publicadores cambiados a CompressedImage
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)
        
        self.bridge = CvBridge()

        # --- VARIABLES DE MEMORIA ---
        self.last_error = 0.0        
        self.left_lane = None       
        self.right_lane = None      

    # ... (fit_lane y draw_lane se mantienen iguales) ...
    def fit_lane(self, points):
        if len(points) < 50: return None
        line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = line.flatten()
        if abs(vy) < 0.1: return None 
        return line.flatten()

    def draw_lane(self, img, lane, color):
        vx, vy, x0, y0 = lane
        h = img.shape[0]
        y1 = int(h * 0.4)
        y2 = h
        x1 = int(x0 + (y1 - y0) * vx / vy)
        x2 = int(x0 + (y2 - y0) * vx / vy)
        cv2.line(img, (x1, y1), (x2, y2), color, 5)
        return x2

    def image_callback(self, msg):
        try:
            orig_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error convirtiendo imagen: {e}")
            return

        frame = cv2.resize(orig_frame, (640, 480))
        h, w, _ = frame.shape
        debug = frame.copy() 

        # 1. LEER PARAMETROS
        umbral_blanco = self.get_parameter('umbral_blanco').value

        corte_pct = self.get_parameter('corte_y_pct').value
        w_top = self.get_parameter('ancho_top').value
        w_bot = self.get_parameter('ancho_bot').value
        alpha = self.get_parameter('suavizado_pct').value / 100.0
        lane_width_virtual = self.get_parameter('ancho_carril_px').value

        # 2. ROI (TRAPECIO)
        corte_y = int(h * (corte_pct / 100.0))
        cx = w // 2 
        polygon = np.array([[(cx - w_bot // 2, h), (cx + w_bot // 2, h), (cx + w_top, corte_y), (cx - w_top, corte_y)]], np.int32)
        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)
        cv2.polylines(debug, polygon, True, (255, 0, 0), 2)

        # 3. FILTRO DE BLANCOS (ESCALA DE GRISES + UMBRAL)
        # Leemos el unico slider que necesitamos
        umbral = self.get_parameter('umbral_blanco').value
        
        # Convertimos a grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Aplicamos el umbral: todo lo que sea mas brillante que 'umbral' se vuelve blanco (255)
        # Lo que sea mas oscuro se vuelve negro (0)
        _, mask_blancos = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)
        
        # Juntamos los blancos detectados con tu ROI (el trapecio)
        mask = cv2.bitwise_and(mask_blancos, mask_roi)

        # 4. EXTRACCION Y LOGICA (Se mantiene igual hasta la publicación)
        ys, xs = np.where(mask > 0)
        
        if len(xs) > 0:
            points = np.column_stack((xs, ys))
            mid = w // 2
            left_pts = points[points[:, 0] < mid]
            right_pts = points[points[:, 0] >= mid]
            new_left = self.fit_lane(left_pts)
            new_right = self.fit_lane(right_pts)

            if new_left is not None:
                self.left_lane = new_left if self.left_lane is None else alpha * new_left + (1 - alpha) * self.left_lane
            if new_right is not None:
                self.right_lane = new_right if self.right_lane is None else alpha * new_right + (1 - alpha) * self.right_lane

            target_x = mid 
            if self.left_lane is not None and self.right_lane is not None:
                xl = self.draw_lane(debug, self.left_lane, (0, 255, 0)) 
                xr = self.draw_lane(debug, self.right_lane, (0, 255, 0)) 
                target_x = (xl + xr) // 2
            elif self.left_lane is not None:
                xl = self.draw_lane(debug, self.left_lane, (0, 0, 255))
                target_x = xl + (lane_width_virtual // 2)
            elif self.right_lane is not None:
                xr = self.draw_lane(debug, self.right_lane, (0, 0, 255)) 
                target_x = xr - (lane_width_virtual // 2)

            error = (target_x - mid) / float(mid)
            error = np.clip(error, -1.0, 1.0)
            smooth_error = 0.7 * self.last_error + 0.3 * error
            self.last_error = smooth_error

            # Visuales
            vis_target = int(mid + smooth_error * mid)
            cv2.circle(debug, (vis_target, int(h*0.8)), 15, (255, 255, 0), -1)
            cv2.line(debug, (mid, int(h*0.5)), (mid, h), (255, 0, 0), 2)
            
            # Publicar Error
            msg_err = Float32()
            msg_err.data = float(smooth_error)
            self.publisher_error.publish(msg_err)

        # 3. PUBLICACIÓN COMPRIMIDA (Fuera del if para que siempre publique flujo)
        # Nota: cv2_to_compressed_imgmsg detecta automáticamente si es mono o color
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
