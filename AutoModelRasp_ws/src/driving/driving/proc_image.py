#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ProcesadorImagenNode(Node):
    def __init__(self):
        super().__init__('proc_image')
        self.get_logger().info("Iniciando Procesador Central de Visión...")

        self.bridge = CvBridge()

        # ==========================================
        # PARÁMETROS DINÁMICOS (Para ajustar en RQT)
        # ==========================================
        # 1. Parámetros para Carriles (Grises/Blancos)
        self.declare_parameter('carril_umbral_blanco', 110)
        
        # 2. Parámetros para Cruces Peatonales (HLS)
        self.declare_parameter('cruce_l_min', 110)
        self.declare_parameter('cruce_s_max', 255)

        # 3. Parámetros para Señales de Stop (HSV - Rojo)
        self.declare_parameter('hsv_h_min1', 0)
        self.declare_parameter('hsv_s_min1', 150)
        self.declare_parameter('hsv_v_min1', 70)
        self.declare_parameter('hsv_h_max1', 10)
        self.declare_parameter('hsv_s_max1', 255)
        self.declare_parameter('hsv_v_max1', 255)

        self.declare_parameter('hsv_h_min2', 160)
        self.declare_parameter('hsv_s_min2', 150)
        self.declare_parameter('hsv_v_min2', 70)
        self.declare_parameter('hsv_h_max2', 179)
        self.declare_parameter('hsv_s_max2', 255)
        self.declare_parameter('hsv_v_max2', 255)

        # ==========================================
        # SUSCRIPCIONES Y PUBLICADORES
        # ==========================================
        # Nos suscribimos SOLAMENTE a la imagen a color
        self.sub_rgb = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.image_callback, 10)

        # Publicamos las 3 máscaras en formato mono8 (muy ligeras)
        self.pub_mask_carril = self.create_publisher(Image, '/vision/mascaras/carril', 10)
        self.pub_mask_cruce = self.create_publisher(Image, '/vision/mascaras/cruce', 10)
        self.pub_mask_stop = self.create_publisher(Image, '/vision/mascaras/stop', 10)

    def image_callback(self, msg):
        try:
            # Convertimos la imagen de ROS a OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")
            return

        # Estandarizamos el tamaño una sola vez para todos
        frame = cv2.resize(frame, (640, 480))

        # ==========================================
        # 1. MÁSCARA PARA CARRILES (Escala de grises)
        # ==========================================
        umbral_carril = self.get_parameter('carril_umbral_blanco').value
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask_carril = cv2.threshold(gray, umbral_carril, 255, cv2.THRESH_BINARY)

        # ==========================================
        # 2. MÁSCARA PARA CRUCE PEATONAL (HLS)
        # ==========================================
        l_min = self.get_parameter('cruce_l_min').value
        s_max = self.get_parameter('cruce_s_max').value
        
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        _, L, S = cv2.split(hls)
        mask_cruce = ((L > l_min) & (S < s_max)).astype(np.uint8) * 255
        
        # Limpieza morfológica rápida
        kernel_cruce = np.ones((3, 3), np.uint8)
        mask_cruce = cv2.morphologyEx(mask_cruce, cv2.MORPH_OPEN, kernel_cruce)

        # ==========================================
        # 3. MÁSCARA PARA STOP (HSV - Rojos)
        # ==========================================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Leemos los rangos rojos
        l_red1 = np.array([self.get_parameter('hsv_h_min1').value, self.get_parameter('hsv_s_min1').value, self.get_parameter('hsv_v_min1').value])
        u_red1 = np.array([self.get_parameter('hsv_h_max1').value, self.get_parameter('hsv_s_max1').value, self.get_parameter('hsv_v_max1').value])
        
        l_red2 = np.array([self.get_parameter('hsv_h_min2').value, self.get_parameter('hsv_s_min2').value, self.get_parameter('hsv_v_min2').value])
        u_red2 = np.array([self.get_parameter('hsv_h_max2').value, self.get_parameter('hsv_s_max2').value, self.get_parameter('hsv_v_max2').value])

        mask_stop1 = cv2.inRange(hsv, l_red1, u_red1)
        mask_stop2 = cv2.inRange(hsv, l_red2, u_red2)
        mask_stop = cv2.bitwise_or(mask_stop1, mask_stop2)

        kernel_stop = np.ones((5, 5), np.uint8)
        mask_stop = cv2.morphologyEx(mask_stop, cv2.MORPH_CLOSE, kernel_stop)

        # ==========================================
        # PUBLICAR LAS MÁSCARAS
        # ==========================================
        # Las publicamos como mono8. Esto es importantísimo para que la red local
        # no se sature y los nodos especialistas gasten ~0% de CPU en decodificarlas.
        self.pub_mask_carril.publish(self.bridge.cv2_to_imgmsg(mask_carril, encoding="mono8"))
        self.pub_mask_cruce.publish(self.bridge.cv2_to_imgmsg(mask_cruce, encoding="mono8"))
        self.pub_mask_stop.publish(self.bridge.cv2_to_imgmsg(mask_stop, encoding="mono8"))

def main(args=None):
    rclpy.init(args=args)
    node = ProcesadorImagenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
