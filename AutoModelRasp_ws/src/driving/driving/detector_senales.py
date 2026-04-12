#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
import struct

# ==============================================================================
# DETECTOR DE SEÑALES (Muestreo por Máscara Exacta + Respaldo Matemático)
# ==============================================================================

class DetectorSenalesNode(Node):
    def __init__(self):
        super().__init__('detector_senales')
        self.get_logger().info('Iniciando Detector de Señales (Máscara Exacta + Math Fallback)')
        self.bridge = CvBridge()
        
        # --- PARÁMETROS DINÁMICOS (RQT) ---
        # Filtros HSV
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 15)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('v_max', 255)
        
        # Geometría para el Respaldo Matemático
        self.declare_parameter('focal_length', 320.0) 
        self.declare_parameter('ancho_real_senal', 0.18) 

        # Memorias de sensores
        self.latest_point_cloud = None
        self.distancia_final = 0.0

        # --- SUSCRIPCIONES ---
        self.sub_rgb = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.rgb_callback, 1)
        
        self.sub_points = self.create_subscription(
            PointCloud2, '/ascamera_hp60c/camera_publisher/depth0/points', self.points_callback, 1)
        
        # --- PUBLICADORES ---
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/calibracion_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/calibracion_resultado/compressed', 10)
        self.pub_distancia = self.create_publisher(Float32, '/vision/senal_stop/distancia_nube', 10)

    def points_callback(self, msg):
        self.latest_point_cloud = msg

    def get_distancia_por_mascara(self, contorno):
        """Extrae Z leyendo exclusivamente los píxeles dentro del contorno rojo."""
        if self.latest_point_cloud is None: return None
        msg = self.latest_point_cloud
        
        # Creamos una imagen negra y rellenamos el contorno de blanco
        mascara_exacta = np.zeros((480, 640), dtype=np.uint8)
        cv2.drawContours(mascara_exacta, [contorno], -1, 255, -1)
        
        # Obtenemos las coordenadas de los píxeles blancos
        ys, xs = np.where(mascara_exacta == 255)
        if len(xs) == 0: return None
        
        # Muestreo: Leemos máximo ~50-60 píxeles para no sobrecargar el CPU
        step = max(1, len(xs) // 60) 
        puntos_z = []

        for i in range(0, len(xs), step):
            # Escalamiento de coordenadas
            u = int(xs[i] * (msg.width / 640.0))
            v = int(ys[i] * (msg.height / 480.0))
            
            pixel_offset = (v * msg.row_step) + (u * msg.point_step)
            try:
                z_bytes = msg.data[pixel_offset + 8 : pixel_offset + 12]
                z = struct.unpack('f', bytes(z_bytes))[0]
                
                # Ignoramos el ruido cercano del chasis (<= 0.15m) y los NaNs
                if not np.isnan(z) and not np.isinf(z) and z > 0.15:
                    puntos_z.append(z)
            except: 
                continue
        
        # Usamos la mediana para obtener una distancia hiper-estable
        if puntos_z:
            return float(np.median(puntos_z))
            
        return None

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        frame = cv2.resize(frame, (640, 480))
        debug = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 1. LEER PARÁMETROS RQT
        lower = np.array([
            self.get_parameter('h_min').value, 
            self.get_parameter('s_min').value, 
            self.get_parameter('v_min').value
        ])
        upper = np.array([
            self.get_parameter('h_max').value, 
            self.get_parameter('s_max').value, 
            self.get_parameter('v_max').value
        ])
        
        # 2. MÁSCARA Y DILATACIÓN
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=2)

        # 3. CONTORNOS
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contornos:
            if cv2.contourArea(cnt) > 200:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                if 6 <= len(approx) <= 12:
                    x, y, w, h = cv2.boundingRect(approx)
                    # Filtro de forma (Casi cuadrado = Señal de frente)
                    if 0.7 < float(w)/h < 1.3:
                        
                        # --- 4. EXTRACCIÓN DE DISTANCIA ---
                        dist = self.get_distancia_por_mascara(approx)
                        metodo = "PC"
                        
                        # Si PointCloud falla (Gazebo transparente) o marca distancias irreales
                        if dist is None or dist < 0.15:
                            f_len = self.get_parameter('focal_length').value
                            w_real = self.get_parameter('ancho_real_senal').value
                            
                            # Modelo Matemático Pinhole
                            dist = (w_real * f_len) / float(w)
                            metodo = "Math"
                        
                        self.distancia_final = dist
                        self.pub_distancia.publish(Float32(data=self.distancia_final))

                        # --- 5. VISUALIZACIÓN EN CALIBRACION RESULTADO ---
                        # Contorno Exacto en magenta
                        cv2.drawContours(debug, [approx], 0, (255, 0, 255), 2)
                        
                        # Texto con el método usado (PC o Math), la Distancia y el Ancho en px (w)
                        dist_txt = f"{metodo}: {self.distancia_final:.2f}m (w:{w}px)"
                        cv2.putText(debug, dist_txt, (x, y - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        
                        # Alerta visual de frenado
                        if self.distancia_final <= 0.40:
                            cv2.putText(debug, "FRENANDO!", (x, y + h + 30), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)

        # 6. PUBLICAR IMÁGENES
        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(mask))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(debug))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorSenalesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()