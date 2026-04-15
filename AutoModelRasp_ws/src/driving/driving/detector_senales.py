#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
import math

class DetectorSenalesNode(Node):
    def __init__(self):
        super().__init__('detector_senales_geometrico')
        self.bridge = CvBridge()

        # --- CONFIGURACIÓN FÍSICA DEL VEHÍCULO ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('h_camara', 0.30),  # Altura 'h' desde el suelo a la lente (m)
                ('dist_cam_defensa', 0.15),  # Distancia eje x entre cámara y defensa (m)
                ('focal_length', 320.0),  # Calibración intrínseca
                ('ancho_real_senal', 0.18),  # Ancho señal Stop estándar competencia
                ('area_minima', 400)
            ]
        )

        # Memoria para la última imagen de profundidad recibida
        self.latest_depth_image = None

        # --- SUSCRIPCIONES Y PUBLICADORES ---
        # 1. Nos suscribimos a la máscara roja procesada (blanco y negro)
        self.sub_mask = self.create_subscription(
            Image, '/vision/mascaras/stop', self.mask_callback, 1)
            
        # 2. Nos suscribimos a la imagen de profundidad en crudo (16 bits)
        self.sub_depth = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw', self.depth_callback, 1)

        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/resultado_geometrico/compressed', 10)
        self.pub_dist_frenado = self.create_publisher(Float32, '/vision/distancia_real', 10)

    def depth_callback(self, msg):
        try:
            # Decodificar profundidad real (16 bits, milímetros)
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception as e:
            self.get_logger().error(f"Error procesando profundidad: {e}")

    def calcular_geometria_real(self, d_visual):
        """Aplica Pitágoras y offsets para saber cuánto le falta a la DEFENSA."""
        h = self.get_parameter('h_camara').value
        d_offset = self.get_parameter('dist_cam_defensa').value

        # Evitar errores matemáticos si la lectura es menor a la altura
        if d_visual <= h:
            return 0.0

        # Pitágoras para obtener distancia proyectada al suelo (cateto)
        d_suelo = math.sqrt(d_visual ** 2 - h ** 2)

        # Restar lo que el carro tiene de "nariz" después de la cámara
        d_real_frenado = d_suelo - d_offset
        return max(0.0, d_real_frenado)

    def mask_callback(self, msg):
        try:
            # Recibimos la imagen mono8 (blanco y negro puro)
            mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
            mask = cv2.resize(mask, (640, 480))
        except:
            return

        # Creamos una imagen a color para el debug visual
        frame_debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contornos:
            if cv2.contourArea(cnt) < self.get_parameter('area_minima').value: continue

            x, y, w, h_rect = cv2.boundingRect(cnt)

            # --- OBTENER DISTANCIA VISUAL ---
            d_visual = None
            
            # Intentar usar el sensor 3D (Depth Image)
            if self.latest_depth_image is not None:
                # Muestreo central
                u, v = int(x + w / 2), int(y + h_rect / 2)
                
                # Asegurar que el punto esté dentro de las dimensiones de la imagen
                dh, dw = self.latest_depth_image.shape
                if 0 <= u < dw and 0 <= v < dh:
                    dist_mm = self.latest_depth_image[v, u]
                    if dist_mm > 0: # La cámara Astra a veces manda 0 si hay ruido o mucho brillo
                        d_visual = dist_mm / 1000.0 # Convertir mm a metros

            # Fallback Pinhole (Si falla la cámara de profundidad o manda 0)
            if d_visual is None or np.isnan(d_visual):
                d_visual = (self.get_parameter('ancho_real_senal').value * self.get_parameter('focal_length').value) / w

            # --- CÁLCULO GEOMÉTRICO FINAL ---
            dist_frenado = self.calcular_geometria_real(d_visual)
            self.pub_dist_frenado.publish(Float32(data=dist_frenado))

            # Visualización
            color = (0, 0, 255) if dist_frenado < 0.3 else (0, 255, 0)
            cv2.rectangle(frame_debug, (x, y), (x + w, y + h_rect), color, 2)
            cv2.putText(frame_debug, f"Defensa a: {dist_frenado:.2f}m", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(frame_debug))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorSenalesNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()