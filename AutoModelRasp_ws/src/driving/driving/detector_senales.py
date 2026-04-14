#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
import struct
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

        # Rangos HSV para Rojo (OpenCV)
        self.LOWER_RED1 = np.array([0, 150, 70])
        self.UPPER_RED1 = np.array([10, 255, 255])
        self.LOWER_RED2 = np.array([160, 150, 70])
        self.UPPER_RED2 = np.array([179, 255, 255])

        self.latest_point_cloud = None

        # Suscripciones y Publicadores
        self.sub_rgb = self.create_subscription(Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.rgb_callback,
                                                1)
        self.sub_points = self.create_subscription(PointCloud2, '/ascamera_hp60c/camera_publisher/depth0/points',
                                                   self.points_callback, 1)

        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/resultado_geometrico/compressed', 10)
        self.pub_dist_frenado = self.create_publisher(Float32, '/vision/distancia_real', 10)

    def points_callback(self, msg):
        self.latest_point_cloud = msg

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

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (640, 480))
        except:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Limpieza morfológica
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contornos:
            if cv2.contourArea(cnt) < self.get_parameter('area_minima').value: continue

            x, y, w, h_rect = cv2.boundingRect(cnt)

            # --- OBTENER DISTANCIA VISUAL (Hipotenusa) ---
            d_visual = None
            if self.latest_point_cloud:
                # Muestreo central
                u, v = int((x + w / 2)), int((y + h_rect / 2))
                offset = (v * self.latest_point_cloud.row_step) + (u * self.latest_point_cloud.point_step)
                try:
                    z_bytes = self.latest_point_cloud.data[offset + 8: offset + 12]
                    d_visual = struct.unpack('f', z_bytes)[0]
                except:
                    pass

            # Fallback Pinhole si falla la nube
            if d_visual is None or np.isnan(d_visual):
                d_visual = (self.get_parameter('ancho_real_senal').value * self.get_parameter('focal_length').value) / w

            # --- CÁLCULO GEOMÉTRICO FINAL ---
            dist_frenado = self.calcular_geometria_real(d_visual)
            self.pub_dist_frenado.publish(Float32(data=dist_frenado))

            # Visualización
            color = (0, 0, 255) if dist_frenado < 0.3 else (0, 255, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h_rect), color, 2)
            cv2.putText(frame, f"Defensa a: {dist_frenado:.2f}m", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(frame))


def main(args=None):
    rclpy.init(args=args)
    node = DetectorSenalesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()