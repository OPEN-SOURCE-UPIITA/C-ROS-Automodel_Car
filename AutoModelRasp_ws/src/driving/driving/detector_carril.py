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
        self.get_logger().info("Detector PRO MAX: Sliding Windows + Sanity Check + Fallback + Density Filter")

        # --- 1. PARÁMETROS DE PERSPECTIVA ---
        self.declare_parameter('umbral_blanco', 180)
        self.declare_parameter('roi_y_top', 260)     
        self.declare_parameter('roi_w_top', 150)     
        self.declare_parameter('roi_w_bot', 600)     
        self.declare_parameter('corte_horizonte_pct', 35) 
        self.declare_parameter('corte_lados_px', 80)      # Aumentado por defecto para evitar los bordes

        # --- 2. PARÁMETROS DE VENTANAS DESLIZANTES ---
        self.declare_parameter('n_windows', 9)       
        self.declare_parameter('window_margin', 60)  
        self.declare_parameter('min_pixels', 40)     
        # NUEVO: Filtro de densidad (Tu idea). Max pixeles blancos permitidos en una caja.
        self.declare_parameter('max_pixels', 2000)   

        # --- 3. PARÁMETROS DE SUPERVIVENCIA Y CORDURA ---
        self.declare_parameter('suavizado_pct', 20)       
        self.declare_parameter('max_curvatura', 0.003)    
        self.declare_parameter('confianza_un_carril', 0.7)
        self.declare_parameter('tolerancia_ancho', 80) 

        self.subscription = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.image_callback, 1) 
        
        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)
        
        self.bridge = CvBridge()

        self.last_error = 0.0       
        self.left_fit = None    
        self.right_fit = None   
        self.lane_width_pixels = 380.0 

    def image_callback(self, msg):
        try: frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        cx = w // 2

        # LEER PARÁMETROS
        umbral = self.get_parameter('umbral_blanco').value
        y_top = self.get_parameter('roi_y_top').value
        w_top = self.get_parameter('roi_w_top').value
        w_bot = self.get_parameter('roi_w_bot').value
        corte_horiz_pct = self.get_parameter('corte_horizonte_pct').value
        corte_lados = self.get_parameter('corte_lados_px').value
        nwindows = self.get_parameter('n_windows').value
        margin = self.get_parameter('window_margin').value
        minpix = self.get_parameter('min_pixels').value
        maxpix = self.get_parameter('max_pixels').value # NUEVO
        alpha = self.get_parameter('suavizado_pct').value / 100.0
        max_curv = self.get_parameter('max_curvatura').value
        tolerancia = self.get_parameter('tolerancia_ancho').value

        # --- PASO 2: BIRD'S EYE VIEW ---
        src_points = np.float32([[cx - w_top//2, y_top], [cx + w_top//2, y_top], 
                                 [cx + w_bot//2, h - 10], [cx - w_bot//2, h - 10]])
        offset = 150 
        dst_points = np.float32([[offset, 0], [w - offset, 0], [w - offset, h], [offset, h]])

        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(frame, M, (w, h), flags=cv2.INTER_LINEAR)

        # --- PASO 3: UMBRAL Y ANTEOJERAS ---
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        _, binary_warped = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)

        limite_superior = int(h * (corte_horiz_pct / 100.0))
        binary_warped[0:limite_superior, :] = 0
        binary_warped[:, 0:corte_lados] = 0
        binary_warped[:, w-corte_lados:w] = 0

        out_img = np.dstack((binary_warped, binary_warped, binary_warped))
        cv2.line(out_img, (0, limite_superior), (w, limite_superior), (0, 0, 255), 2)

        # --- PASO 4 y 5: SLIDING WINDOWS ---
        histogram = np.sum(binary_warped[h//2:, :], axis=0)
        midpoint = int(histogram.shape[0] // 2)
        
        leftx_base = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint]) > 50 else None
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint if np.max(histogram[midpoint:]) > 50 else None

        nonzero = binary_warped.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
        
        left_lane_inds, right_lane_inds = [], []
        window_height = int(h // nwindows)

        leftx_current, rightx_current = leftx_base, rightx_base

        for window in range(nwindows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            if win_y_high < limite_superior: break
                
            if leftx_current is not None:
                win_xleft_low, win_xleft_high = leftx_current - margin, leftx_current + margin
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2) 
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                                  (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                
                # NUEVO: FILTRO DE GROSOR / DENSIDAD
                if len(good_left_inds) < maxpix:
                    left_lane_inds.append(good_left_inds)
                    if len(good_left_inds) > minpix: leftx_current = int(np.mean(nonzerox[good_left_inds]))
                else:
                    cv2.putText(out_img, "MANCHON IGNORADO", (win_xleft_low, win_y_low), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

            if rightx_current is not None:
                win_xright_low, win_xright_high = rightx_current - margin, rightx_current + margin
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2) 
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                                   (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
                
                # NUEVO: FILTRO DE GROSOR / DENSIDAD
                if len(good_right_inds) < maxpix:
                    right_lane_inds.append(good_right_inds)
                    if len(good_right_inds) > minpix: rightx_current = int(np.mean(nonzerox[good_right_inds]))
                else:
                    cv2.putText(out_img, "MANCHON IGNORADO", (win_xright_low, win_y_low), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        # --- PASO 6: AJUSTE MATEMÁTICO ---
        l_fit_raw, r_fit_raw = None, None
        
        if len(left_lane_inds) > 0:
            left_lane_inds = np.concatenate(left_lane_inds)
            leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
            out_img[lefty, leftx] = [0, 0, 255]
            if len(lefty) > 50: 
                l_fit_raw = np.polyfit(lefty, leftx, 2)
                if abs(l_fit_raw[0]) > max_curv: l_fit_raw = None 

        if len(right_lane_inds) > 0:
            right_lane_inds = np.concatenate(right_lane_inds)
            rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]
            out_img[righty, rightx] = [255, 0, 0]
            if len(righty) > 50: 
                r_fit_raw = np.polyfit(righty, rightx, 2)
                if abs(r_fit_raw[0]) > max_curv: r_fit_raw = None

        # --- PASO 6.5: EL FILTRO DE CORDURA ---
        if l_fit_raw is not None and r_fit_raw is not None:
            l_base = l_fit_raw[0]*h**2 + l_fit_raw[1]*h + l_fit_raw[2]
            r_base = r_fit_raw[0]*h**2 + r_fit_raw[1]*h + r_fit_raw[2]
            ancho_detectado = r_base - l_base
            
            if abs(ancho_detectado - self.lane_width_pixels) > tolerancia:
                if self.left_fit is not None and self.right_fit is not None:
                    l_base_vieja = self.left_fit[0]*h**2 + self.left_fit[1]*h + self.left_fit[2]
                    r_base_vieja = self.right_fit[0]*h**2 + self.right_fit[1]*h + self.right_fit[2]
                    
                    diff_izq = abs(l_base - l_base_vieja)
                    diff_der = abs(r_base - r_base_vieja)
                    
                    if diff_izq > diff_der:
                        l_fit_raw = None
                        cv2.putText(out_img, "RECHAZO IZQ", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    else:
                        r_fit_raw = None
                        cv2.putText(out_img, "RECHAZO DER", (w-250, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    l_fit_raw = None 

        # --- PASO 7: SUAVIZADO INTELIGENTE ---
        if l_fit_raw is not None:
            self.left_fit = l_fit_raw if self.left_fit is None else (1-alpha)*l_fit_raw + alpha*self.left_fit
        if r_fit_raw is not None:
            self.right_fit = r_fit_raw if self.right_fit is None else (1-alpha)*r_fit_raw + alpha*self.right_fit

        # --- PASO 8: LOOKAHEAD + SINGLE LANE FALLBACK ---
        y_eval = max(int(h * 0.65), limite_superior + 20)
        tx_l, tx_r = None, None

        if self.left_fit is not None:
            tx_l = self.left_fit[0]*y_eval**2 + self.left_fit[1]*y_eval + self.left_fit[2]
        if self.right_fit is not None:
            tx_r = self.right_fit[0]*y_eval**2 + self.right_fit[1]*y_eval + self.right_fit[2]

        target_x = cx
        confianza = 1.0

        if tx_l is not None and tx_r is not None:
            target_x = (tx_l + tx_r) / 2.0
            self.lane_width_pixels = 0.95 * self.lane_width_pixels + 0.05 * (tx_r - tx_l)
        elif tx_l is not None:
            target_x = tx_l + (self.lane_width_pixels / 2.0)
            confianza = self.get_parameter('confianza_un_carril').value
        elif tx_r is not None:
            target_x = tx_r - (self.lane_width_pixels / 2.0)
            confianza = self.get_parameter('confianza_un_carril').value

        cv2.circle(out_img, (int(target_x), y_eval), 15, (0, 255, 255), -1)

        # --- CÁLCULO FINAL DE ERROR ---
        error_bruto = (target_x - cx) / float(cx)
        error_final = np.clip(error_bruto * confianza, -1.0, 1.0)
        self.last_error = 0.7 * self.last_error + 0.3 * error_final

        cv2.line(out_img, (cx, h), (int(target_x), y_eval), (0, 255, 255), 4)
        cv2.line(out_img, (cx, h), (cx, 0), (255, 255, 255), 1)

        self.publisher_error.publish(Float32(data=float(self.last_error)))
        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(binary_warped))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(out_img))

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