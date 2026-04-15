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
# DETECTOR DE CRUCES PEATONALES PRO (Proyección Horizontal + FFT)
# ==============================================================================

class DetectorCrucesNode(Node):
    def __init__(self):
        super().__init__('detector_cruces')
        self.get_logger().info("Detector de Cruces Peatonales (FFT) Iniciado")

        # =============================
        # PARÁMETROS DINÁMICOS (RQT)
        # =============================
        self.declare_parameter('resize_w', 320)
        self.declare_parameter('resize_h', 240)

        # 1. ROI: Banda de detección (Horizontal)
        self.declare_parameter('roi_y_sup_pct', 60) # Inicio de la banda (ej. 60% de la pantalla)
        self.declare_parameter('roi_y_inf_pct', 10) # Fin de la banda (ej. 10% desde abajo)
        self.declare_parameter('corte_lados_px', 40) # Ignorar bordes de la pista

        # 2. Filtros de Color HLS
        self.declare_parameter('l_min', 180)
        self.declare_parameter('s_max', 100)

        # 3. Parámetros Matemáticos FFT (Frecuencia de la Cebra)
        # ¿Cuántas rayas blancas (ciclos) esperamos ver en la cámara? Normalmente entre 3 y 8
        self.declare_parameter('fft_frecuencia_min', 3) 
        self.declare_parameter('fft_frecuencia_max', 10)
        
        # Este es el umbral clave. Si el pico de la FFT supera esto, frena.
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
        
        pct_sup = self.get_parameter('roi_y_sup_pct').value
        pct_inf = self.get_parameter('roi_y_inf_pct').value
        corte_lados = self.get_parameter('corte_lados_px').value

        l_min = self.get_parameter('l_min').value
        s_max = self.get_parameter('s_max').value
        
        f_min = self.get_parameter('fft_frecuencia_min').value
        f_max = self.get_parameter('fft_frecuencia_max').value
        umbral_fft = self.get_parameter('fft_umbral_magnitud').value
        
        frames_memoria = self.get_parameter('frames_memoria').value

        # =============================
        # 2. PREPROCESAMIENTO Y ROI
        # =============================
        frame = cv2.resize(frame, (w_r, h_r))
        h, w = frame.shape[:2]

        y_sup = int(h * (pct_sup / 100.0))
        y_inf = h - int(h * (pct_inf / 100.0))
        
        # Extraemos la banda de interés
        roi = frame[y_sup:y_inf, corte_lados:w-corte_lados]
        if roi.size == 0: return

        # =============================
        # 3. FILTRO HLS Y MÁSCARA BINARIA
        # =============================
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        _, L, S = cv2.split(hls)
        
        mask = (L > l_min) & (S < s_max)
        mask = mask.astype(np.uint8) * 255
        
        # Limpieza básica
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # =========================================================
        # 4. EL NÚCLEO MATEMÁTICO: PROYECCIÓN (PH) Y FFT
        # =========================================================
        # Paso A: Proyección Horizontal (Sumar píxeles por columnas)
        ph = np.sum(mask, axis=0) / 255.0 # Normalizamos de 0 a Altura del ROI
        
        # Paso B: Centrar la señal (Quitar la media para eliminar la frecuencia 0)
        ph_centered = ph - np.mean(ph)
        
        # Paso C: Transformada Rápida de Fourier (FFT)
        fft_vals = np.fft.fft(ph_centered)
        fft_mag = np.abs(fft_vals)
        
        # Solo necesitamos la primera mitad del espectro (Límite de Nyquist)
        half_N = len(fft_mag) // 2
        fft_mag = fft_mag[:half_N]

        # Paso D: Analizar las frecuencias válidas de la cebra
        f_max = min(f_max, half_N - 1)
        valid_mags = fft_mag[f_min:f_max]
        
        pico_maximo = np.max(valid_mags) if len(valid_mags) > 0 else 0
        cruce_detectado_matematicamente = pico_maximo > umbral_fft

        # =============================
        # 5. CÁLCULO DE DISTANCIA Y PERSISTENCIA
        # =============================
        distancia = 999.0
        debug = frame.copy()
        
        # Dibujar la caja del ROI
        cv2.rectangle(debug, (corte_lados, y_sup), (w-corte_lados, y_inf), (255, 0, 0), 2)

        if cruce_detectado_matematicamente:
            self.contador_persistencia = frames_memoria
            
            # Buscar exactamente en qué píxel Y empieza el cruce dentro del ROI
            # (Escaneamos de abajo hacia arriba en la máscara para hallar el borde)
            y_borde_local = mask.shape[0] - 1
            for y_scan in range(mask.shape[0]-1, 0, -2):
                if np.sum(mask[y_scan, :]) > 255 * 10: # Si hay algo de blanco
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
        # 6. GRAFICADOR VISUAL DE LA FFT (Para RQT)
        # =============================
        # Dibujamos una mini gráfica en la esquina superior derecha del debug
        graf_w, graf_h = 100, 50
        cv2.rectangle(debug, (w - graf_w - 10, 10), (w - 10, 10 + graf_h), (0, 0, 0), -1)
        if len(fft_mag) > 0:
            max_visual = max(umbral_fft * 1.5, np.max(fft_mag)) # Escala automática
            for i, mag in enumerate(fft_mag[:20]): # Mostrar las primeras 20 frecuencias
                x = w - graf_w - 10 + int((i / 20) * graf_w)
                h_bar = int((mag / max_visual) * graf_h)
                color_bar = (0, 255, 0) if f_min <= i <= f_max else (100, 100, 100)
                if i >= f_min and i <= f_max and mag > umbral_fft: color_bar = (0, 0, 255) # Pico rojo!
                cv2.line(debug, (x, 10 + graf_h), (x, 10 + graf_h - h_bar), color_bar, 2)

        # Reconstruir máscara para publicarla
        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[y_sup:y_inf, corte_lados:w-corte_lados] = mask
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