#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
import struct

class DetectorSenalesNode(Node):
    def __init__(self):
        super().__init__('detector_senales_node')
        self.bridge = CvBridge()
        self.get_logger().info("🛑 Detector de Señales (Visión Directa) Iniciado")

        # --- CONFIGURACIÓN FÍSICA DINÁMICA (RQT) ---
        self.declare_parameter('h_camara', 0.30) 
        self.declare_parameter('dist_cam_defensa', 0.15) 
        self.declare_parameter('focal_length', 320.0) 
        self.declare_parameter('ancho_real_senal', 0.18) 
        self.declare_parameter('area_minima', 400)

        # Rangos HSV para Rojo (Señal STOP)
        self.LOWER_RED1 = np.array([0, 150, 70])
        self.UPPER_RED1 = np.array([10, 255, 255])
        self.LOWER_RED2 = np.array([160, 150, 70])
        self.UPPER_RED2 = np.array([179, 255, 255])

        self.latest_point_cloud = None

        # Suscripciones y Publicadores
        self.sub_rgb = self.create_subscription(Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.rgb_callback, 1)
        self.sub_points = self.create_subscription(PointCloud2, '/ascamera_hp60c/camera_publisher/depth0/points', self.points_callback, 1)

        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/resultado_geometrico/compressed', 10)
        self.pub_dist_frenado = self.create_publisher(Float32, '/vision/distancia_real', 10)

    def points_callback(self, msg):
        self.latest_point_cloud = msg

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Forzamos tamaño conocido para la imagen 2D
            img_w, img_h = 640, 480
            frame = cv2.resize(frame, (img_w, img_h))
        except Exception as e:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Limpieza morfológica para evitar ruiditos rojos
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contornos:
            if cv2.contourArea(cnt) < self.get_parameter('area_minima').value: continue

            x, y, w, h_rect = cv2.boundingRect(cnt)

            # Variables para el cálculo
            d_visual = None
            metodo = ""

            # ---------------------------------------------------------
            # MÉTODO A: INTENTAR LEER PROFUNDIDAD Z DE LA NUBE DE PUNTOS
            # ---------------------------------------------------------
            if self.latest_point_cloud:
                # Centroide de la caja delimitadora
                u_img = x + (w / 2.0)
                v_img = y + (h_rect / 2.0)
                
                # Mapeo de resoluciones (de 640x480 a la resolución real de la nube)
                pc_w = self.latest_point_cloud.width
                pc_h = self.latest_point_cloud.height
                
                u_real = int((u_img / img_w) * pc_w)
                v_real = int((v_img / img_h) * pc_h)
                
                # Proteger índices
                u_real = max(0, min(u_real, pc_w - 1))
                v_real = max(0, min(v_real, pc_h - 1))

                offset = (v_real * self.latest_point_cloud.row_step) + (u_real * self.latest_point_cloud.point_step)
                
                try:
                    # Extraer bytes correspondientes a Z (Float32)
                    z_bytes = self.latest_point_cloud.data[offset + 8: offset + 12]
                    z_val = struct.unpack('f', z_bytes)[0]
                    
                    if not math.isnan(z_val) and not math.isinf(z_val) and z_val > 0.05:
                        d_visual = z_val
                        metodo = "3D"
                except Exception:
                    pass

            # ---------------------------------------------------------
            # MÉTODO B: FALLBACK PINHOLE (Geometría por tamaño de píxeles)
            # ---------------------------------------------------------
            if d_visual is None:
                ancho_real = self.get_parameter('ancho_real_senal').value
                focal = self.get_parameter('focal_length').value
                d_visual = (ancho_real * focal) / float(w)
                metodo = "Pinhole"

            # ---------------------------------------------------------
            # CÁLCULO FINAL DE FRENADO (Geometría Directa)
            # ---------------------------------------------------------
            # Como Z (o Pinhole) ya es una línea recta hacia el frente, 
            # ya no usamos Pitágoras. Solo restamos la nariz del carro.
            offset_defensa = self.get_parameter('dist_cam_defensa').value
            dist_frenado = max(0.0, d_visual - offset_defensa)

            # Publicar al cerebro del carro
            self.pub_dist_frenado.publish(Float32(data=dist_frenado))

            # --- DIBUJAR EN PANTALLA ---
            color = (0, 0, 255) if dist_frenado < 0.4 else (0, 255, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h_rect), color, 2)
            
            # Texto informativo con el método usado
            texto = f"Defensa: {dist_frenado:.2f}m [{metodo}]"
            cv2.putText(frame, texto, (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Publicar imagen final para RQT
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(frame))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorSenalesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()