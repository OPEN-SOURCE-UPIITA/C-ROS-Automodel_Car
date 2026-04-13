import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraFilterNode(Node):
    def __init__(self):
        super().__init__('camera_filter')
        
        # Herramienta para traducir imágenes de ROS a OpenCV y viceversa
        self.bridge = CvBridge()
        
        # 1. Nos suscribimos a la cámara que sale de Gazebo
        self.sub = self.create_subscription(
            Image, 
            '/camera/image', 
            self.image_callback, 
            10
        )
        
        # 2. Publicamos la imagen con el nombre EXACTO de tu cámara fisica
        self.pub = self.create_publisher(
            Image, 
            '/ascamera_hp60c/camera_publisher/rgb0/image', 
            10
        )
        
        self.get_logger().info("Filtro de visión activo: Ocultando cofre y simulando Asense HP60C...")

    def image_callback(self, msg):
        try:
            # Convertir el mensaje de ROS a una matriz de OpenCV (NumPy)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Obtener dimensiones
            height, width = cv_image.shape[:2]
            
            # Calcular el punto de corte (El 25% inferior)
            punto_de_corte = int(height * 1)
            
            # Aplicar el filtro: Pintar de negro absoluto la parte inferior
            cv_image[punto_de_corte:height, 0:width] = [0, 0, 0]
            
            # Convertir la imagen procesada de vuelta a formato ROS 2
            filtered_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # Esto mantiene la estampa de tiempo sincronizada.
            filtered_msg.header = msg.header
            # Disfrazamos el frame_id para que TF2 crea que es la cámara física
            filtered_msg.header.frame_id = 'ascamera_link' 
            
            # Publicar la imagen lista para tus algoritmos
            self.pub.publish(filtered_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main():
    rclpy.init()
    rclpy.spin(CameraFilterNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()