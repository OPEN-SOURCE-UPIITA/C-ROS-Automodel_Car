#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DemoRebase(Node):
    def __init__(self):
        super().__init__('demo_rebase')
        self.get_logger().info("Iniciando Nodo de Rebase: MODO VERDAD ABSOLUTA")

        # --- PARÁMETROS RQT ---
        self.declare_parameter('roi_x', 140)  
        self.declare_parameter('roi_y', 200)  
        self.declare_parameter('roi_w', 360)  
        self.declare_parameter('roi_h', 100)  
        self.declare_parameter('distancia_alerta', 0.50) 
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.bridge = CvBridge()

        # --- SUSCRIPCIONES Y PUBLICACIONES ---
        self.sub_depth = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw', self.depth_callback, 10)
        
        self.pub_bandera = self.create_publisher(Int32, '/vision/bandera_rebase', 10)
        self.pub_visual = self.create_publisher(CompressedImage, '/vision/rebase_visual/compressed', 10)

        self.estado_previo = 0

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def depth_callback(self, msg):
        try: 
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception as e: 
            return

        h, w = depth_image.shape

        roi_x = self.get_parameter('roi_x').value
        roi_y = self.get_parameter('roi_y').value
        roi_w = self.get_parameter('roi_w').value
        roi_h = self.get_parameter('roi_h').value
        umbral = self.get_parameter('distancia_alerta').value

        roi_x = max(0, min(roi_x, w - 1))
        roi_y = max(0, min(roi_y, h - 1))
        roi_w = min(roi_w, w - roi_x)
        roi_h = min(roi_h, h - roi_y)

        zona_interes = depth_image[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]
        
        ancho_franja = roi_w // 3
        
        distancias = []
        porcentajes_muertos = []

        # --- ANÁLISIS DE LA CRUDA REALIDAD ---
        for i in range(3):
            x_inicio = i * ancho_franja
            x_fin = (i + 1) * ancho_franja
            franja_actual = zona_interes[:, x_inicio:x_fin]
            
            total_pixeles = franja_actual.size
            pixeles_validos = franja_actual[franja_actual > 0]
            cantidad_validos = len(pixeles_validos)
            
            # Calculamos cuántos píxeles están muertos (valen 0)
            ceros = total_pixeles - cantidad_validos
            porcentaje = (ceros / total_pixeles) * 100.0
            porcentajes_muertos.append(porcentaje)
            
            # Si al menos un píxel sirve, sacamos promedio. Si no, forzamos a 0.00
            if cantidad_validos > 0:
                dist_m = np.mean(pixeles_validos) / 1000.0
            else:
                dist_m = 0.00 # 100% Ciego
                
            distancias.append(dist_m)

        dist_izq = distancias[0]
        dist_cen = distancias[1]
        dist_der = distancias[2]

        # --- LÓGICA DE DECISIÓN BÁSICA ---
        # Si la cámara tira 0.00m (ciega), no activará el obstáculo, solo activará si la lectura es real y menor al umbral
        ocupado_izq = (dist_izq > 0.01) and (dist_izq < umbral)
        ocupado_cen = (dist_cen > 0.01) and (dist_cen < umbral)
        ocupado_der = (dist_der > 0.01) and (dist_der < umbral)

        nuevo_estado = 0 

        if ocupado_cen and ocupado_izq and ocupado_der:
            nuevo_estado = 1  
        elif ocupado_cen and not ocupado_izq and not ocupado_der:
            nuevo_estado = 6  
        elif ocupado_cen and ocupado_izq and not ocupado_der:
            nuevo_estado = 6  
        elif ocupado_cen and ocupado_der and not ocupado_izq:
            nuevo_estado = 7  

        msg_estado = Int32()
        msg_estado.data = nuevo_estado
        self.pub_bandera.publish(msg_estado)

        if nuevo_estado != self.estado_previo:
            self.estado_previo = nuevo_estado

        # --- VISUALIZACIÓN RQT (El modo Rayos X) ---
        visual_8bit = cv2.convertScaleAbs(depth_image, alpha=(255.0/3000.0))
        visual_color = cv2.applyColorMap(visual_8bit, cv2.COLORMAP_JET)

        nombres_zonas = ["IZQ", "CEN", "DER"]
        
        for i in range(3):
            x1 = roi_x + (i * ancho_franja)
            x2 = roi_x + ((i + 1) * ancho_franja)
            dist = distancias[i]
            z_porc = porcentajes_muertos[i]
            
            # Rojo solo si realmente hay un obstáculo validado
            color_caja = (0, 0, 255) if (dist > 0.01 and dist < umbral) else (0, 255, 0)
            
            # Dibujar rectángulo
            cv2.rectangle(visual_color, (x1, roi_y), (x2, roi_y + roi_h), color_caja, 2)
            
            # Imprimir Distancia y Porcentaje de Ceros
            texto = f"{dist:.2f}m (Z:{z_porc:.0f}%)"
            
            cv2.putText(visual_color, f"{nombres_zonas[i]}:", (x1 + 5, roi_y + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(visual_color, texto, (x1 + 5, roi_y + 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        self.pub_visual.publish(self.bridge.cv2_to_compressed_imgmsg(visual_color))

def main(args=None):
    rclpy.init(args=args)
    node = DemoRebase()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()