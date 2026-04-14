#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class RadarFranjasNode(Node):
    def __init__(self):
        super().__init__('radar_franjas')
        self.get_logger().info("Iniciando Radar 3D: 5 Franjas de Profundidad")

        # --- PARÁMETROS RQT PARA LA ZONA DE ESCANEO (ROI) ---
        self.declare_parameter('roi_x', 170)  # Coordenada X (centrado aprox en 640x480)
        self.declare_parameter('roi_y', 200)  # Coordenada Y
        self.declare_parameter('roi_w', 300)  # Ancho total de la caja (debe ser divisible entre 5)
        self.declare_parameter('roi_h', 100)  # Alto de la caja
        
        # Umbral de peligro: A cuántos metros el auto decide cambiar de carril
        self.declare_parameter('distancia_alerta', 0.30) 
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.bridge = CvBridge()

        # --- SUBSCRIPCIONES Y PUBLICACIONES ---
        self.sub_depth = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw', self.depth_callback, 10)
        
        # Este es el tópico que activará el cambio de carril en tu nodo de motores
        self.pub_alerta = self.create_publisher(Bool, '/radar/alerta_rebase', 10)
        
        self.pub_visual = self.create_publisher(CompressedImage, '/radar/visual/compressed', 10)

        self.estado_alerta_previo = False

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def depth_callback(self, msg):
        try: 
            # Decodificar profundidad real (16 bits)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception as e: 
            return

        h, w = depth_image.shape

        # Leer parámetros
        roi_x = self.get_parameter('roi_x').value
        roi_y = self.get_parameter('roi_y').value
        roi_w = self.get_parameter('roi_w').value
        roi_h = self.get_parameter('roi_h').value
        distancia_alerta = self.get_parameter('distancia_alerta').value

        # Evitar recortes fuera de la imagen
        roi_x = max(0, min(roi_x, w - 1))
        roi_y = max(0, min(roi_y, h - 1))
        roi_w = min(roi_w, w - roi_x)
        roi_h = min(roi_h, h - roi_y)

        # Recorte de la zona matemática
        zona_interes = depth_image[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]
        
        # Dividir la zona en 5 franjas verticales
        ancho_franja = roi_w // 5
        promedios_m = []

        for i in range(5):
            x_inicio = i * ancho_franja
            x_fin = (i + 1) * ancho_franja
            franja_actual = zona_interes[:, x_inicio:x_fin]
            
            # Filtrar ceros (ruido de la cámara Astra)
            pixeles_validos = franja_actual[franja_actual > 0]
            
            if len(pixeles_validos) > 0:
                dist_m = np.mean(pixeles_validos) / 1000.0 # Convertir mm a metros
            else:
                dist_m = 9.99 # Sin obstáculo (Infinito)
                
            promedios_m.append(dist_m)

        # --- LÓGICA DE DETECCIÓN (Prioridad al centro) ---
        # La franja 2 es exactamente el centro del auto [0, 1, (2), 3, 4]
        distancia_centro = promedios_m[2]
        alerta_activa = False

        if distancia_centro < distancia_alerta:
            alerta_activa = True

        # Publicar la alerta solo si cambió de estado (para no saturar la red)
        if alerta_activa != self.estado_alerta_previo:
            msg_alerta = Bool()
            msg_alerta.data = alerta_activa
            self.pub_alerta.publish(msg_alerta)
            self.estado_alerta_previo = alerta_activa
            
            if alerta_activa:
                self.get_logger().warn(f"¡OBSTÁCULO A {distancia_centro:.2f}m! Mandando orden de rebase.")

        # --- VISUALIZACIÓN RQT ---
        # Mapa de color para ver la profundidad
        visual_8bit = cv2.convertScaleAbs(depth_image, alpha=(255.0/4000.0))
        visual_color = cv2.applyColorMap(visual_8bit, cv2.COLORMAP_JET)

        # Dibujar las 5 franjas
        for i in range(5):
            x1 = roi_x + (i * ancho_franja)
            x2 = roi_x + ((i + 1) * ancho_franja)
            dist = promedios_m[i]
            
            # Color: Rojo si está muy cerca, Verde si está libre
            color_caja = (0, 0, 255) if dist < distancia_alerta else (0, 255, 0)
            
            cv2.rectangle(visual_color, (x1, roi_y), (x2, roi_y + roi_h), color_caja, 2)
            
            # Mostrar la distancia encima de cada franja (un poco apretado pero útil)
            texto = f"{dist:.1f}" if dist < 9.0 else "INF"
            cv2.putText(visual_color, texto, (x1 + 5, roi_y + roi_h // 2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        self.pub_visual.publish(self.bridge.cv2_to_compressed_imgmsg(visual_color))

def main(args=None):
    rclpy.init(args=args)
    node = RadarFranjasNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
