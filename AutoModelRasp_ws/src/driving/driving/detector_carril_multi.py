#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorMultiCarrilNode(Node):
    def __init__(self):
        super().__init__('detector_carril_multi')
        self.get_logger().info("Iniciando Detector Multi-Carril (3 Líneas)")

        # --- 1. PARÁMETROS DE PERSPECTIVA ---
        self.declare_parameter('roi_y_top', 260)     
        self.declare_parameter('roi_w_top', 150)     
        self.declare_parameter('roi_w_bot', 600)     
        self.declare_parameter('corte_horizonte_pct', 35) 
        self.declare_parameter('corte_lados_px', 80)

        # --- 2. PARÁMETROS DE VENTANAS ---
        self.declare_parameter('n_windows', 9)       
        self.declare_parameter('window_margin', 50)  
        self.declare_parameter('min_pixels', 40)     
        self.declare_parameter('max_pixels', 2000)   

        # --- 3. PARÁMETROS DE CONTROL ---
        self.declare_parameter('suavizado_pct', 20)        
        self.declare_parameter('tolerancia_ancho', 100)

        # --- SUBS/PUBS ---
        # SE SUSCRIBE A LA MÁSCARA YA PROCESADA
        self.subscription = self.create_subscription(
            Image, '/vision/mascaras/carril', self.image_callback, 1) 
        
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/lane_detection/compressed', 10)
        self.pub_carril = self.create_publisher(String, '/vision/carril_actual', 10)
        
        self.bridge = CvBridge()

        # Almacenamiento de polinomios (L, C, R)
        self.left_fit = None    
        self.center_fit = None
        self.right_fit = None   
        self.lane_width_pixels = 250.0 

    def image_callback(self, msg):
        try:
            # Recibe la imagen mono8 (blanco y negro puro)
            frame_binario = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except:
            return

        frame_binario = cv2.resize(frame_binario, (640, 480))
        h, w = frame_binario.shape[:2]
        cx = w // 2

        # LEER PARÁMETROS
        y_top = self.get_parameter('roi_y_top').value
        w_top = self.get_parameter('roi_w_top').value
        w_bot = self.get_parameter('roi_w_bot').value
        nwindows = self.get_parameter('n_windows').value
        margin = self.get_parameter('window_margin').value
        minpix = self.get_parameter('min_pixels').value
        maxpix = self.get_parameter('max_pixels').value
        alpha = self.get_parameter('suavizado_pct').value / 100.0

        # --- PASO 1: BIRD'S EYE VIEW ---
        src_points = np.float32([[cx - w_top//2, y_top], [cx + w_top//2, y_top], 
                                 [cx + w_bot//2, h - 10], [cx - w_bot//2, h - 10]])
        offset = 150 
        dst_points = np.float32([[offset, 0], [w - offset, 0], [w - offset, h], [offset, h]])

        M = cv2.getPerspectiveTransform(src_points, dst_points)
        binary_warped = cv2.warpPerspective(frame_binario, M, (w, h), flags=cv2.INTER_LINEAR)

        # --- PASO 2: LIMPIEZA DE BORDES ---
        binary_warped[:, 0:50] = 0
        binary_warped[:, w-50:w] = 0

        # --- PASO 3: HISTOGRAMA TRIPLE ---
        histogram = np.sum(binary_warped[h//2:, :], axis=0)
        tercio = w // 3
        
        leftx_base = np.argmax(histogram[:tercio]) if np.max(histogram[:tercio]) > 50 else None
        centerx_base = np.argmax(histogram[tercio:2*tercio]) + tercio if np.max(histogram[tercio:2*tercio]) > 50 else None
        rightx_base = np.argmax(histogram[2*tercio:]) + 2*tercio if np.max(histogram[2*tercio:]) > 50 else None

        # --- PASO 4: VENTANAS DESLIZANTES ---
        out_img = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR) # Convertir a BGR para dibujar a color
        nonzero = binary_warped.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
        
        window_height = h // nwindows
        l_inds, c_inds, r_inds = [], [], []
        curr_l, curr_c, curr_r = leftx_base, centerx_base, rightx_base

        img_disponible = binary_warped.copy()

        for window in range(nwindows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height

            # 1. VENTANA DERECHA 
            if curr_r is not None:
                xr_low, xr_high = curr_r - margin, curr_r + margin
                cv2.rectangle(out_img, (xr_low, win_y_low), (xr_high, win_y_high), (0, 0, 255), 2)
                
                zona_r = img_disponible[win_y_low:win_y_high, xr_low:xr_high]
                
                if np.any(zona_r > 0):
                    ids = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= xr_low) & (nonzerox < xr_high)).nonzero()[0]
                    
                    if minpix < len(ids) < maxpix:
                        r_inds.append(ids)
                        curr_r = int(np.mean(nonzerox[ids]))
                        img_disponible[win_y_low:win_y_high, xr_low:xr_high] = 0

            # 2. VENTANA CENTRAL 
            if curr_c is not None:
                xc_low, xc_high = curr_c - margin, curr_c + margin
                cv2.rectangle(out_img, (xc_low, win_y_low), (xc_high, win_y_high), (255, 255, 0), 2)
                
                zona_c = img_disponible[win_y_low:win_y_high, xc_low:xc_high]
                
                if np.any(zona_c > 0):
                    ids = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= xc_low) & (nonzerox < xc_high)).nonzero()[0]
                    
                    if minpix < len(ids) < maxpix:
                        c_inds.append(ids)
                        curr_c = int(np.mean(nonzerox[ids]))
                        img_disponible[win_y_low:win_y_high, xc_low:xc_high] = 0

            # 3. VENTANA IZQUIERDA 
            if curr_l is not None:
                xl_low, xl_high = curr_l - margin, curr_l + margin
                cv2.rectangle(out_img, (xl_low, win_y_low), (xl_high, win_y_high), (0, 255, 0), 2)
                
                zona_l = img_disponible[win_y_low:win_y_high, xl_low:xl_high]
                
                if np.any(zona_l > 0):
                    ids = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= xl_low) & (nonzerox < xl_high)).nonzero()[0]
                    
                    if minpix < len(ids) < maxpix:
                        l_inds.append(ids)
                        curr_l = int(np.mean(nonzerox[ids]))

        # --- PASO 5: AJUSTE POLINOMIAL Y SUAVIZADO ---
        def get_fit(indices):
            if len(indices) > 0:
                idx = np.concatenate(indices)
                return np.polyfit(nonzeroy[idx], nonzerox[idx], 2)
            return None

        fL_raw, fC_raw, fR_raw = get_fit(l_inds), get_fit(c_inds), get_fit(r_inds)

        if fL_raw is not None: self.left_fit = fL_raw if self.left_fit is None else (1-alpha)*fL_raw + alpha*self.left_fit
        if fC_raw is not None: self.center_fit = fC_raw if self.center_fit is None else (1-alpha)*fC_raw + alpha*self.center_fit
        if fR_raw is not None: self.right_fit = fR_raw if self.right_fit is None else (1-alpha)*fR_raw + alpha*self.right_fit

        # --- PASO 6: LÓGICA DE CARRIL ACTUAL ---
        y_eval = h - 50 
        carril_actual = "DESCONOCIDO"
        
        if self.center_fit is not None:
            xC = np.polyval(self.center_fit, y_eval)
            dist_izq = abs(cx - (np.polyval(self.left_fit, y_eval) + xC)/2) if self.left_fit is not None else 1000
            dist_der = abs(cx - (xC + np.polyval(self.right_fit, y_eval))/2) if self.right_fit is not None else 1000
            
            if dist_izq < dist_der and dist_izq < 150: carril_actual = "IZQUIERDO"
            elif dist_der < dist_izq and dist_der < 150: carril_actual = "DERECHO"

        # --- PASO 7: VISUALIZACIÓN ---
        if self.left_fit is not None and self.center_fit is not None:
            self.draw_lane_area(out_img, self.left_fit, self.center_fit, (0, 255, 0)) 
        if self.center_fit is not None and self.right_fit is not None:
            self.draw_lane_area(out_img, self.center_fit, self.right_fit, (255, 255, 0)) 

        cv2.putText(out_img, f"CARRIL: {carril_actual}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        self.pub_carril.publish(String(data=carril_actual))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(out_img))

    def draw_lane_area(self, img, fit1, fit2, color):
        y_pts = np.linspace(0, img.shape[0]-1, 20)
        x1_pts = np.polyval(fit1, y_pts)
        x2_pts = np.polyval(fit2, y_pts)
        pts1 = np.array([np.transpose(np.vstack([x1_pts, y_pts]))])
        pts2 = np.array([np.flipud(np.transpose(np.vstack([x2_pts, y_pts])))])
        pts = np.hstack((pts1, pts2))
        overlay = img.copy()
        cv2.fillPoly(overlay, np.int_([pts]), color)
        cv2.addWeighted(overlay, 0.3, img, 0.7, 0, dst=img)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorMultiCarrilNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()