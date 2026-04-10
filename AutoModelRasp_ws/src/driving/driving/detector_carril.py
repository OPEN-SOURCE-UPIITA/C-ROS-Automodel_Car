#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorCarrilNode(Node):
    def __init__(self):
        super().__init__('detector_carril')

        self.get_logger().info("Iniciando Detector de Carril PRO (Bird's Eye View + Sliding Windows + Lookahead)")

        # --- PARAMETROS DINAMICOS (RQT) ---
        # 1. Umbral para la escala de grises
        self.declare_parameter('umbral_blanco', 180)
        
        # 2. Parametros para la deformacion de perspectiva (Trapecio)
        self.declare_parameter('roi_y_top', 260)     
        self.declare_parameter('roi_w_top', 150)     
        self.declare_parameter('roi_w_bot', 600)     
        
        # 3. Parametros de Ventanas Deslizantes
        self.declare_parameter('n_windows', 9)       
        self.declare_parameter('window_margin', 60)  
        self.declare_parameter('min_pixels', 40)     

        # --- SUSCRIPCIONES Y PUBLICADORES ---
        self.subscription = self.create_subscription(
            Image, 
            '/ascamera_hp60c/camera_publisher/rgb0/image', 
            self.image_callback, 
            1) 
        
        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)
        
        self.bridge = CvBridge()

        # --- MEMORIA DEL SISTEMA ---
        self.last_error = 0.0       
        self.left_fit_hist = None    
        self.right_fit_hist = None   

    def image_callback(self, msg):
        try:
            orig_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error convirtiendo imagen: {e}")
            return

        frame = cv2.resize(orig_frame, (640, 480))
        h, w = frame.shape[:2]

        # ---------------------------------------------------------
        # PASO 1: LEER PARAMETROS
        # ---------------------------------------------------------
        umbral = self.get_parameter('umbral_blanco').value
        y_top = self.get_parameter('roi_y_top').value
        w_top = self.get_parameter('roi_w_top').value
        w_bot = self.get_parameter('roi_w_bot').value
        nwindows = self.get_parameter('n_windows').value
        margin = self.get_parameter('window_margin').value
        minpix = self.get_parameter('min_pixels').value

        # ---------------------------------------------------------
        # PASO 2: BIRD'S EYE VIEW
        # ---------------------------------------------------------
        cx = w // 2
        
        src_points = np.float32([
            [cx - w_top // 2, y_top], 
            [cx + w_top // 2, y_top], 
            [cx + w_bot // 2, h - 10], 
            [cx - w_bot // 2, h - 10]  
        ])

        offset = 150 
        dst_points = np.float32([
            [offset, 0], 
            [w - offset, 0], 
            [w - offset, h], 
            [offset, h]
        ])

        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(frame, M, (w, h), flags=cv2.INTER_LINEAR)

        # ---------------------------------------------------------
        # PASO 3: ESCALA DE GRISES Y UMBRAL
        # ---------------------------------------------------------
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        _, binary_warped = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)

        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        # ---------------------------------------------------------
        # PASO 4: HISTOGRAMA
        # ---------------------------------------------------------
        histogram = np.sum(binary_warped[h//2:, :], axis=0)
        
        midpoint = int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # ---------------------------------------------------------
        # PASO 5: VENTANAS DESLIZANTES
        # ---------------------------------------------------------
        window_height = int(h // nwindows)
        
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2) 
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2) 
            
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        out_img[lefty, leftx] = [0, 0, 255]
        out_img[righty, rightx] = [255, 0, 0]

        # ---------------------------------------------------------
        # PASO 6: AJUSTE POLINOMIAL
        # ---------------------------------------------------------
        try:
            if len(lefty) > 0:
                self.left_fit_hist = np.polyfit(lefty, leftx, 2)
            if len(righty) > 0:
                self.right_fit_hist = np.polyfit(righty, rightx, 2)
        except Exception as e:
            self.get_logger().error("Fallo al calcular curva matematicamente")

        # ---------------------------------------------------------
        # PASO 7: CALCULO DEL ERROR (STEERING CON LOOKAHEAD)
        # ---------------------------------------------------------
        target_x = cx # Por defecto apunta al centro
        
        if self.left_fit_hist is not None and self.right_fit_hist is not None:
            l_fit = self.left_fit_hist
            r_fit = self.right_fit_hist
            
            # --- EL PUNTO DE MIRA (LOOKAHEAD) ---
            # Miramos al 65% de la altura de la pantalla (mas adelante)
            y_eval = int(h * 0.65) 
            
            # Evaluamos la curva en ese nuevo punto adelantado
            left_eval_x = l_fit[0]*y_eval**2 + l_fit[1]*y_eval + l_fit[2]
            right_eval_x = r_fit[0]*y_eval**2 + r_fit[1]*y_eval + r_fit[2]
            
            # El centro objetivo ahora esta adelantado a la curva
            target_x = (left_eval_x + right_eval_x) / 2.0
            
            # Dibujamos un circulo rojo para ver el Lookahead Point
            cv2.circle(out_img, (int(target_x), y_eval), 15, (0, 0, 255), -1)

        # Calcular error normalizado (-1 a 1)
        error = (target_x - cx) / float(cx)
        error = np.clip(error, -1.0, 1.0)
        
        # Filtro de suavizado (Pasa-bajas)
        smooth_error = 0.7 * self.last_error + 0.3 * error
        self.last_error = smooth_error

        # Dibujar la linea central objetivo
        cv2.line(out_img, (cx, h), (int(target_x), int(h*0.65)), (0, 255, 255), 4)
        cv2.line(out_img, (cx, h), (cx, 0), (255, 255, 255), 1)

        # ---------------------------------------------------------
        # PASO 8: PUBLICACION
        # ---------------------------------------------------------
        msg_err = Float32()
        msg_err.data = float(smooth_error)
        self.publisher_error.publish(msg_err)

        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(binary_warped))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(out_img))


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