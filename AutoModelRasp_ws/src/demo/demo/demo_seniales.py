#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Importamos la librería de YOLO
from ultralytics import YOLO

class SenalesYOLO(Node):
    def __init__(self):
        super().__init__('demo_seniales_yolo')
        
        # 1. Buscar y cargar el modelo
        # Esto busca best.pt en la misma carpeta donde está este script de Python
        model_path = os.path.join(os.path.dirname(__file__), 'best.pt')
        self.get_logger().info(f"Cargando cerebro YOLO desde: {model_path}")
        self.yolo = YOLO(model_path)
        
        self.bridge = CvBridge()
        
        # 2. Suscriptor: Los "ojos" del carrito (Tu cámara Astra)
        self.sub_camara = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.camera_callback,
            10
        )
        
        # 3. Publicador: Las instrucciones a los motores
        self.pub_bandera = self.create_publisher(Int32, '/vision/bandera_senales', 10)
        
        # 4. Publicador de Debugging: La vista para RQT con las cajas dibujadas
        self.pub_imagen_debug = self.create_publisher(Image, '/vision/senales_debug', 10)
        
        self.get_logger().info("Nodo YOLO Iniciado. ¡Esperando imágenes de la pista!")

    def camera_callback(self, msg):
        # A. Convertir la imagen de ROS a formato OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error de cv_bridge: {e}")
            return

        # B. Pasarle la foto a YOLO (conf=0.6 significa "solo avísame si estás 60% seguro")
        resultados = self.yolo.predict(source=cv_image, conf=0.6, verbose=False)
        
        bandera = 0 # 0 = Conducción Normal (Nada detectado)
        
        # C. Lógica si YOLO encuentra algo en la imagen
        if len(resultados[0].boxes) > 0:
            # Tomamos la caja principal detectada
            mejor_caja = resultados[0].boxes[0]
            clase_id = int(mejor_caja.cls[0].item())
            nombre_clase = self.yolo.names[clase_id] # Extrae el nombre (ej. "stop")
            
            # Mapeo: Convertimos el nombre a un número para tu lógica de control
            if nombre_clase == "stop":
                bandera = 1
            elif nombre_clase == "escuela":
                bandera = 2
            elif nombre_clase == "giro":
                bandera = 3
            elif nombre_clase == "derrape":
                bandera = 4
            elif nombre_clase == "kilometros":
                bandera = 5
                
            self.get_logger().info(f"¡Señal {nombre_clase} detectada! Bandera enviada: {bandera}")

        # D. Publicar el estado a los motores
        msg_estado = Int32()
        msg_estado.data = bandera
        self.pub_bandera.publish(msg_estado)

        # E. Magia para RQT: Dibujar las cajas y publicar la imagen de vuelta a ROS
        # results[0].plot() le pinta los cuadros y nombres de colores automáticamente
        imagen_anotada = resultados[0].plot() 
        try:
            img_msg_debug = self.bridge.cv2_to_imgmsg(imagen_anotada, encoding='bgr8')
            self.pub_imagen_debug.publish(img_msg_debug)
        except Exception as e:
            self.get_logger().error(f"Error publicando debug: {e}")

def main(args=None):
    rclpy.init(args=args)
    nodo = SenalesYOLO()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()