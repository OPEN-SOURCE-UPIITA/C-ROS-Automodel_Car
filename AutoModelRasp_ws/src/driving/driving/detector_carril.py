#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorCarrilNode(Node):
    def __init__(self):
        super().__init__('detector_carril')

        self.get_logger().info("Iniciando Detector de Carril (Parametros RQT)")

        # --- PARAMETROS DINAMICOS (Reemplazan a los sliders de OpenCV) ---
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 179)
        self.declare_parameter('s_min', 0)
        self.declare_parameter('s_max', 60)   # Saturacion baja para buscar blanco
        self.declare_parameter('v_min', 150)  # Brillo alto
        self.declare_parameter('v_max', 255)

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
        
        # Publicadores separados para verlos comodamente en rqt_image_view
        self.pub_mascara = self.create_publisher(Image, '/vision/carril_mascara', 10)
        self.pub_resultado = self.create_publisher(Image, '/vision/carril_resultado', 10)
        
        self.bridge = CvBridge()

        # --- VARIABLES DE MEMORIA ---
        self.last_error = 0.0       
        self.left_lane = None       
        self.right_lane = None      

    def fit_lane(self, points):
        """ Ajusta linea recta a puntos blancos. Retorna [vx, vy, x0, y0] """
        if len(points) < 50: 
            return None
        line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = line.flatten()
        if abs(vy) < 0.1: 
            return None 
        return line.flatten()

    def draw_lane(self, img, lane, color):
        """ Dibuja linea y retorna interseccion inferior X """
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

        # 1. LEER PARAMETROS DE RQT
        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        s_max = self.get_parameter('s_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value

        corte_pct = self.get_parameter('corte_y_pct').value
        w_top = self.get_parameter('ancho_top').value
        w_bot = self.get_parameter('ancho_bot').value
        
        alpha = self.get_parameter('suavizado_pct').value / 100.0
        lane_width_virtual = self.get_parameter('ancho_carril_px').value

        # 2. ROI (TRAPECIO)
        corte_y = int(h * (corte_pct / 100.0))
        cx = w // 2 
        polygon = np.array([[
            (cx - w_bot // 2, h), (cx + w_bot // 2, h),
            (cx + w_top, corte_y), (cx - w_top, corte_y)
        ]], np.int32)

        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)
        cv2.polylines(debug, polygon, True, (255, 0, 0), 2)

        # 3. COLOR (HSV)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_color = cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))
        mask = cv2.bitwise_and(mask_color, mask_roi)

        # 4. EXTRACCION DE CARRIL
        ys, xs = np.where(mask > 0)
        
        if len(xs) == 0:
            # Si no hay lineas, publicar de todos modos para evitar congelamientos en rqt
            self.pub_mascara.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.pub_resultado.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            return

        points = np.column_stack((xs, ys))
        mid = w // 2
        
        left_pts = points[points[:, 0] < mid]
        right_pts = points[points[:, 0] >= mid]

        new_left = self.fit_lane(left_pts)
        new_right = self.fit_lane(right_pts)

        # 5. MEMORIA Y SUAVIZADO DE LINEAS
        if new_left is not None:
            self.left_lane = new_left if self.left_lane is None else alpha * new_left + (1 - alpha) * self.left_lane
        
        if new_right is not None:
            self.right_lane = new_right if self.right_lane is None else alpha * new_right + (1 - alpha) * self.right_lane

        # 6. CALCULO DEL OBJETIVO (TARGET)
        target_x = mid 
        if self.left_lane is not None and self.right_lane is not None:
            xl = self.draw_lane(debug, self.left_lane, (0, 255, 0)) 
            xr = self.draw_lane(debug, self.right_lane, (0, 255, 0)) 
            target_x = (xl + xr) // 2
            cv2.line(debug, (xl, h-50), (xr, h-50), (0, 255, 255), 2)
        elif self.left_lane is not None:
            xl = self.draw_lane(debug, self.left_lane, (0, 0, 255))
            target_x = xl + (lane_width_virtual // 2)
        elif self.right_lane is not None:
            xr = self.draw_lane(debug, self.right_lane, (0, 0, 255)) 
            target_x = xr - (lane_width_virtual // 2)

        # 7. CALCULO DEL ERROR Y PUBLICACION
        error = (target_x - mid) / float(mid)
        error = np.clip(error, -1.0, 1.0)
        smooth_error = 0.7 * self.last_error + 0.3 * error
        self.last_error = smooth_error

        # Visuales
        vis_target = int(mid + smooth_error * mid)
        cv2.circle(debug, (vis_target, int(h*0.8)), 15, (255, 255, 0), -1)
        cv2.line(debug, (mid, int(h*0.5)), (mid, h), (255, 0, 0), 2)

        # Publicar topicos separados
        self.pub_mascara.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        self.pub_resultado.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        
        msg_err = Float32()
        msg_err.data = float(smooth_error)
        self.publisher_error.publish(msg_err)


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