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
# DETECTOR DE CRUCES PEATONALES PRO (Polígono Trapezoidal + FFT)
# ==============================================================================

class DetectorCrucesNode(Node):
    def __init__(self):
        super().__init__('detector_cruces')
        self.get_logger().info("Detector de Cruces Peatonales (Trapecio + FFT) Iniciado")

        # =============================
        # PARÁMETROS DINÁMICOS (RQT)
        # =============================
        self.declare_parameter('resize_w', 320)
        self.declare_parameter('resize_h', 240)

        # 1. ROI TRAPEZOIDAL (Porcentajes de la pantalla)
        self.declare_parameter('roi_y_sup_pct', 78)  # Altura superior (ej. 55% desde arriba)
        self.declare_parameter('roi_y_inf_pct', 95)  # Altura inferior (ej. 95% desde arriba)
        self.declare_parameter('roi_w_sup_pct', 50)  # Ancho de la línea superior (ej. 40% del ancho total)
        self.declare_parameter('roi_w_inf_pct', 100) # Ancho de la línea inferior (ej. 100% del ancho total)

        # 2. Filtros de Color HLS
        self.declare_parameter('l_min', 110)
        self.declare_parameter('s_max', 100)

        # 3. Parámetros Matemáticos FFT (Frecuencia de la Cebra)
        self.declare_parameter('fft_frecuencia_min', 4) 
        self.declare_parameter('fft_frecuencia_max', 10)
        self.declare_parameter('fft_umbral_magnitud', 500.0) 

        # 4. Anti-Parpadeo
        self.declare_parameter('frames_memoria', 10)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription = self.create_subscription(Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.image_callback, 10)
        self.pub_distancia = self.create_publisher(Float32, '/vision/cruce_peatonal/distancia', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/vision/cruce_peatonal/debug/compressed', 10)
        self.pub_mask = self.create_publisher(CompressedImage, '/vision/cruce_peatonal/mask/compressed', 10)

        self.bridge = CvBridge()
        self.contador_persistencia = 0
        self.ultima_y_global = 999.0

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try: frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # =============================
        # 1. LEER PARÁMETROS
        # =============================
        w_r = self.get_parameter('resize_w').value
        h_r = self.get_parameter('resize_h').value
        
        y_sup_pct = self.get_parameter('roi_y_sup_pct').value / 100.0
        y_inf_pct = self.get_parameter('roi_y_inf_pct').value / 100.0
        w_sup_pct = self.get_parameter('roi_w_sup_pct').value / 100.0
        w_inf_pct = self.get_parameter('roi_w_inf_pct').value / 100.0

        l_min = self.get_parameter('l_min').value
        s_max = self.get_parameter('s_max').value
        
        f_min = self.get_parameter('fft_frecuencia_min').value
        f_max = self.get_parameter('fft_frecuencia_max').value
        umbral_fft = self.get_parameter('fft_umbral_magnitud').value
        frames_memoria = self.get_parameter('frames_memoria').value

        # =============================
        # 2. PREPROCESAMIENTO Y GEOMETRÍA DEL TRAPECIO
        # =============================
        frame = cv2.resize(frame, (w_r, h_r))
        h, w = frame.shape[:2]

        y_sup = int(h * y_sup_pct)
        y_inf = int(h * y_inf_pct)
        
        # Calcular los vértices del trapecio (Centrados en la pantalla)
        x_center = w // 2
        w_sup_half = int((w * w_sup_pct) / 2)
        w_inf_half = int((w * w_inf_pct) / 2)

        pt1 = [x_center - w_sup_half, y_sup] # Arriba Izquierda
        pt2 = [x_center + w_sup_half, y_sup] # Arriba Derecha
        pt3 = [x_center + w_inf_half, y_inf] # Abajo Derecha
        pt4 = [x_center - w_inf_half, y_inf] # Abajo Izquierda
        
        pts_globales = np.array([[pt1, pt2, pt3, pt4]], dtype=np.int32)

        # Cortamos la imagen en "Y" para procesar menos píxeles (Optimización)
        roi = frame[y_sup:y_inf, :]
        if roi.size == 0: return

        # =============================
        # 3. MÁSCARA DEL POLÍGONO Y FILTRO HLS
        # =============================
        # Crear máscara negra del tamaño del ROI
        poly_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        
        # Ajustar los puntos al sistema de coordenadas del ROI cortado
        pts_roi = np.array([[[pt1[0], 0], [pt2[0], 0], [pt3[0], y_inf - y_sup], [pt4[0], y_inf - y_sup]]], dtype=np.int32)
        cv2.fillPoly(poly_mask, pts_roi, 255) # Pintamos el trapecio de blanco

        # Aplicar el filtro de color
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        _, L, S = cv2.split(hls)
        color_mask = (L > l_min) & (S < s_max)
        color_mask = color_mask.astype(np.uint8) * 255

        # Fusionar la forma del trapecio con el filtro de color (Operación AND)
        mask = cv2.bitwise_and(color_mask, poly_mask)
        
        # Limpieza de ruido
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # =========================================================
        # 4. EL NÚCLEO MATEMÁTICO: PROYECCIÓN (PH) Y FFT
        # =========================================================
        ph = np.sum(mask, axis=0) / 255.0 
        ph_centered = ph - np.mean(ph)
        
        fft_vals = np.fft.fft(ph_centered)
        fft_mag = np.abs(fft_vals)
        
        half_N = len(fft_mag) // 2
        fft_mag = fft_mag[:half_N]

        f_max = min(f_max, half_N - 1)
        valid_mags = fft_mag[f_min:f_max]
        
        pico_maximo = np.max(valid_mags) if len(valid_mags) > 0 else 0
        cruce_detectado_matematicamente = pico_maximo > umbral_fft

        # =============================
        # 5. CÁLCULO DE DISTANCIA Y PERSISTENCIA
        # =============================
        distancia = 999.0
        debug = frame.copy()
        
        # 🔥 DIBUJAR EL TRAPECIO EN LA PANTALLA DEBUG
        cv2.polylines(debug, pts_globales, isClosed=True, color=(255, 0, 0), thickness=2)

        if cruce_detectado_matematicamente:
            self.contador_persistencia = frames_memoria
            
            y_borde_local = mask.shape[0] - 1
            for y_scan in range(mask.shape[0]-1, 0, -2):
                if np.sum(mask[y_scan, :]) > 255 * 10: 
                    y_borde_local = y_scan
                    break
            self.ultima_y_global = float(y_sup + y_borde_local)
        else:
            if self.contador_persistencia > 0:
                self.contador_persistencia -= 1

        if self.contador_persistencia > 0:
            distancia = self.ultima_y_global
            color = (0, 0, 255) if cruce_detectado_matematicamente else (0, 165, 255)
            cv2.putText(debug, f"CRUCE! Mag: {int(pico_maximo)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.line(debug, (0, int(self.ultima_y_global)), (w, int(self.ultima_y_global)), color, 3)

        # =============================
        # 6. GRAFICADOR VISUAL DE LA FFT
        # =============================
        graf_w, graf_h = 100, 50
        cv2.rectangle(debug, (w - graf_w - 10, 10), (w - 10, 10 + graf_h), (0, 0, 0), -1)
        if len(fft_mag) > 0:
            max_visual = max(umbral_fft * 1.5, np.max(fft_mag)) 
            for i, mag in enumerate(fft_mag[:20]): 
                x = w - graf_w - 10 + int((i / 20) * graf_w)
                h_bar = int((mag / max_visual) * graf_h)
                color_bar = (0, 255, 0) if f_min <= i <= f_max else (100, 100, 100)
                if i >= f_min and i <= f_max and mag > umbral_fft: color_bar = (0, 0, 255) 
                cv2.line(debug, (x, 10 + graf_h), (x, 10 + graf_h - h_bar), color_bar, 2)

        # Reconstruir máscara para publicarla
        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[y_sup:y_inf, :] = mask
        mask_bgr = cv2.cvtColor(full_mask, cv2.COLOR_GRAY2BGR)

        self.pub_distancia.publish(Float32(data=distancia))
        self.pub_debug.publish(self.bridge.cv2_to_compressed_imgmsg(debug))
        self.pub_mask.publish(self.bridge.cv2_to_compressed_imgmsg(mask_bgr))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorCrucesNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()