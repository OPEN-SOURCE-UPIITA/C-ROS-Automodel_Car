#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

# ==============================================================================
# DETECTOR DE CRUCES PEATONALES (Vista de Águila + Histograma Horizontal)
# ==============================================================================

class DetectorCrucesNode(Node):
    def __init__(self):
        super().__init__('detector_cruces')
        self.get_logger().info("Detector de Cruces Peatonales Iniciado")

        # --- 1. PARÁMETROS DE PERSPECTIVA (Bird's Eye) ---
        self.declare_parameter('umbral_blanco', 180)
        self.declare_parameter('roi_y_top', 260)     
        self.declare_parameter('roi_w_top', 150)     
        self.declare_parameter('roi_w_bot', 600)     
        self.declare_parameter('corte_lados_px', 150) # Ignorar bordes de la pista
        
        # --- 2. PARÁMETROS DEL CRUCE ---
        # ¿Cuántos píxeles blancos en una fila se necesitan para considerarla una "línea de cebra"?
        self.declare_parameter('ancho_min_linea_cruce', 120) 
        # ¿Cuántas líneas gruesas juntas confirman que es un paso de cebra?
        self.declare_parameter('num_lineas_para_cruce', 3)   
        # Coordenada Y en la vista de águila donde el auto debe detenerse
        self.declare_parameter('distancia_frenado_y', 350)   

        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- 3. COMUNICACIONES ---
        self.subscription = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.image_callback, 10) 
        
        # Publicamos la "distancia". Si es 999.0, la vía está libre.
        self.pub_distancia_cruce = self.create_publisher(Float32, '/vision/cruce_peatonal/distancia', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/cruce_peatonal/resultado/compressed', 10)
        
        self.bridge = CvBridge()

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try: 
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: 
            return

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        cx = w // 2

        # LEER PARÁMETROS
        umbral = self.get_parameter('umbral_blanco').value
        y_top = self.get_parameter('roi_y_top').value
        w_top = self.get_parameter('roi_w_top').value
        w_bot = self.get_parameter('roi_w_bot').value
        corte_lados = self.get_parameter('corte_lados_px').value
        umbral_ancho_linea = self.get_parameter('ancho_min_linea_cruce').value
        lineas_requeridas = self.get_parameter('num_lineas_para_cruce').value
        linea_freno = self.get_parameter('distancia_frenado_y').value

        # --- PASO 1: VISTA DE ÁGUILA (BIRD'S EYE VIEW) ---
        src_points = np.float32([[cx - w_top//2, y_top], [cx + w_top//2, y_top], 
                                 [cx + w_bot//2, h - 10], [cx - w_bot//2, h - 10]])
        offset = 150 
        dst_points = np.float32([[offset, 0], [w - offset, 0], [w - offset, h], [offset, h]])

        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(frame, M, (w, h), flags=cv2.INTER_LINEAR)

        # --- PASO 2: UMBRALIZACIÓN Y LIMPIEZA ---
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        _, binary_warped = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)
        
        # Ignoramos los lados de la imagen para no confundir las líneas del carril con cruces
        binary_warped[:, 0:corte_lados] = 0
        binary_warped[:, w-corte_lados:w] = 0
        
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        # --- PASO 3: HISTOGRAMA HORIZONTAL ---
        # Sumamos la cantidad de píxeles blancos en cada fila (eje Y)
        histograma_horizontal = np.sum(binary_warped == 255, axis=1)

        # --- PASO 4: DETECCIÓN DEL CRUCE ---
        distancia_cruce = 999.0
        lineas_detectadas = 0
        y_cruce_mas_cercano = 0
        en_linea = False
        
        # Escaneamos la imagen desde abajo (más cerca del auto) hacia arriba
        for y in range(h-1, 0, -1):
            pixeles_blancos = histograma_horizontal[y]
            
            # Si hay muchos píxeles blancos en esta fila, es una franja del cruce
            if pixeles_blancos > umbral_ancho_linea:
                if not en_linea:
                    lineas_detectadas += 1
                    en_linea = True
                    # Guardamos la posición Y de la primera raya de cebra que toque el auto
                    if lineas_detectadas == 1:
                        y_cruce_mas_cercano = y
            else:
                # Si los píxeles blancos bajan, significa que estamos en el asfalto negro entre las rayas
                en_linea = False

        # Dibujamos la línea límite donde queremos que el auto se detenga (Naranja)
        cv2.line(out_img, (0, linea_freno), (w, linea_freno), (0, 165, 255), 2)

        # --- PASO 5: EVALUACIÓN ---
        if lineas_detectadas >= lineas_requeridas:
            # Enviamos la coordenada Y de la primera raya. (Mayor Y = más cerca del auto)
            distancia_cruce = float(y_cruce_mas_cercano)
            
            cv2.putText(out_img, f"¡CRUCE DETECTADO! Lineas: {lineas_detectadas}", 
                        (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.line(out_img, (0, y_cruce_mas_cercano), (w, y_cruce_mas_cercano), (0, 0, 255), 4)

        # 6. PUBLICAR
        self.pub_distancia_cruce.publish(Float32(data=distancia_cruce))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(out_img))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorCrucesNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()