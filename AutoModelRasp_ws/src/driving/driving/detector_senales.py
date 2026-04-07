#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage #Para comprimir las imágenes

# ==============================================================================
        # SIGUIENTES PASOS PARA INTEGRACION EN LA COMPETENCIA
        # ==============================================================================
        # 1. CORRECCION TRIGONOMETRICA DE PROFUNDIDAD:
        #    - La camara lee la distancia radial (hipotenusa) al centro de la senal.
        #    - Calcular la distancia horizontal real sobre el suelo (cateto adyacente).
        #    - El auto debe detenerse máxio a 30cm de la señal
        #
        # 2. INTEGRACION CON STM32 (MOTORES):
        #    - Importar los mensajes del motor y hacer el control del valor real avanzado con la distancia restante si es necesario
        #
        # 3. MAQUINA DE ESTADOS (EVITAR BUCLE INFINITO):
        #    - Implementar un temporizador para que se detenga al menos por 5 segundos.
        #    
# ==============================================================================

class DetectorSenalesNode(Node):
    def __init__(self):
        super().__init__('detector_senales')
        
        self.get_logger().info('Iniciando Detector de Senales (HSV + Profundidad)')
        self.bridge = CvBridge()
        
        # --- PARAMETROS DINAMICOS (Para calibrar desde rqt) ---
        # Inicializados en el rango bajo de rojo para evitar problemas de iluminacion
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 15)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('v_max', 255)

        # Memoria para la profundidad
        self.latest_depth_img = None

        # --- SUSCRIPCIONES ---
        self.sub_rgb = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_callback,
            1)

        self.sub_depth = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback,
            1)

        # --- PUBLICADORES ---
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/calibracion_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/calibracion_resultado/compressed', 10)

    def depth_callback(self, msg):
        try:
            # passthrough lee los pixeles tal cual vienen del hardware (usualmente milimetros)
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error en imagen depth: {e}")

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error en imagen RGB: {e}")
            return

        frame = cv2.resize(frame, (640, 480))
        debug = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 1. LEER LOS SLIDERS DE RQT EN TIEMPO REAL
        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        s_max = self.get_parameter('s_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value
        
        # 2. FILTRO DE COLOR
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Limpieza suave para no borrar la senal cuando esta lejos
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # 3. DETECCION DE CONTORNOS
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contornos:
            area = cv2.contourArea(cnt)
            
            # Filtro de ruido pequeno (bajamos a 200 para detectar de mas lejos)
            if area > 200:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                vertices = len(approx)
                
                # Un octagono a lo lejos puede verse como un poligono de 6 a 12 lados
                if 6 <= vertices <= 12 and cv2.isContourConvex(approx):
                    
                    # 4. INVARIANZA DE ESCALA (Relacion de aspecto)
                    # Un octagono de frente siempre es simetrico (cuadrado)
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    
                    # Tolerancia: entre 0.8 y 1.2 significa que es casi tan ancho como alto
                    if 0.8 < aspect_ratio < 1.2:
                        cx = x + w // 2
                        cy = y + h // 2
                        
                        distancia_metros = 0.0
                        
                        # 5. LECTURA DE LA CAMARA DE PROFUNDIDAD
                        if self.latest_depth_img is not None:
                            depth_h, depth_w = self.latest_depth_img.shape
                            scale_x = depth_w / 640.0
                            scale_y = depth_h / 480.0
                            
                            d_x = int(cx * scale_x)
                            d_y = int(cy * scale_y)
                            
                            # Asegurar que no leemos fuera de la matriz
                            d_x = min(max(d_x, 0), depth_w - 1)
                            d_y = min(max(d_y, 0), depth_h - 1)
                            
                            depth_val = self.latest_depth_img[d_y, d_x]
                            distancia_metros = depth_val / 1000.0 # Convertir mm a metros

                        # DIBUJAR DEBUG
                        cv2.drawContours(debug, [approx], 0, (0, 255, 0), 3)
                        cv2.rectangle(debug, (x, y), (x+w, y+h), (255, 0, 0), 2)
                        cv2.putText(debug, "STOP", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        
                        if distancia_metros > 0.0:
                            msg_dist = f"Dist: {distancia_metros:.2f}m"
                            cv2.putText(debug, msg_dist, (x, y + h + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                            
                            # LOGICA DE FRENADO
                            if distancia_metros <= 0.35:
                                cv2.putText(debug, "FRENANDO!", (x, y + h + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
                                # Esto evita que la terminal se congele por spam
                                self.get_logger().warn(f'DISTANCIA CRITICA: {distancia_metros:.2f}m. COMANDO STOP ENVIADO.', throttle_duration_sec=1.0)
        # 6. PUBLICAR RESULTADOS
        # Comprimimos la máscara (como es blanco y negro, el puente de CV se encarga)
        msg_mask = self.bridge.cv2_to_compressed_imgmsg(mask)
        self.pub_mascara.publish(msg_mask)
        # Comprimimos el resultado con las cajas dibujadas
        msg_res = self.bridge.cv2_to_compressed_imgmsg(debug)
        self.pub_resultado.publish(msg_res)


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