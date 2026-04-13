#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from motor_msgs.msg import MotorCommand
from cv_bridge import CvBridge
import cv2
import numpy as np

class PruebaCruceNode(Node):
    def __init__(self):
        super().__init__('prueba_cruce')
        self.get_logger().info("Cazador de Cebras: Modo Zona de Escaneo Visible")

        # --- 0. BOTÓN DE SEGURIDAD ---
        self.declare_parameter('habilitar_conduccion', False)

        # --- 1. PARÁMETROS DE VISIÓN (Base BEV) ---
        self.declare_parameter('umbral_blanco', 150)
        self.declare_parameter('roi_y_top', 300)     
        self.declare_parameter('roi_w_top', 200)     
        self.declare_parameter('roi_w_bot', 800)     

        # --- 2. ZONA DE ESCANEO (Tu idea del cuadro visible) ---
        self.declare_parameter('zona_y_top', 250)        # Límite superior de la caja amarilla
        self.declare_parameter('zona_y_bot', 400)        # Límite inferior de la caja amarilla
        self.declare_parameter('min_area_raya', 400)     # Qué tan grande debe ser la mancha blanca
        self.declare_parameter('min_rayas_cebra', 3)     # Cuántas manchas juntas detonan el freno

        # --- 3. PARÁMETROS DE MOTOR DE PRUEBA ---
        self.declare_parameter('vel_prueba_dc', 40)      
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.image_callback, 10)
        
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/cruce_resultado/compressed', 10)

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try: frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        cx = w // 2

        modo_activo = self.get_parameter('habilitar_conduccion').value
        umbral = self.get_parameter('umbral_blanco').value
        y_top = self.get_parameter('roi_y_top').value
        w_top = self.get_parameter('roi_w_top').value
        w_bot = self.get_parameter('roi_w_bot').value
        
        # Parámetros de la zona
        zona_top = self.get_parameter('zona_y_top').value
        zona_bot = self.get_parameter('zona_y_bot').value
        min_area = self.get_parameter('min_area_raya').value
        min_rayas = self.get_parameter('min_rayas_cebra').value
        vel_prueba = self.get_parameter('vel_prueba_dc').value

        # --- 1. VISTA DE PÁJARO (BEV) ---
        src_points = np.float32([[cx - w_top//2, y_top], [cx + w_top//2, y_top], 
                                 [cx + w_bot//2, h - 10], [cx - w_bot//2, h - 10]])
        offset = 150 
        dst_points = np.float32([[offset, 0], [w - offset, 0], [w - offset, h], [offset, h]])

        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(frame, M, (w, h), flags=cv2.INTER_LINEAR)

        # --- 2. BINARIZACIÓN ---
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)
        out_img = np.dstack((binary, binary, binary))

        # === DIBUJAR LA ZONA DE ESCANEO (EL CUADRO AMARILLO) ===
        # Todo lo que pase fuera de este cuadro, no existe para el programa
        cv2.rectangle(out_img, (0, zona_top), (w, zona_bot), (0, 255, 255), 2)
        cv2.putText(out_img, "ZONA ESCANEO", (10, zona_top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- 3. CAZADOR DE MANCHAS DENTRO DE LA ZONA ---
        contornos, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rayas_validas = 0

        for contorno in contornos:
            x, y, w_rect, h_rect = cv2.boundingRect(contorno)
            cy = y + (h_rect // 2) # Centro de la mancha blanca
            
            # REGLA 1: ¿La mancha está dentro del cuadro amarillo?
            if zona_top < cy < zona_bot:
                area = cv2.contourArea(contorno)
                
                # REGLA 2: ¿Es suficientemente grande para ser una raya de cebra?
                if area > min_area:
                    rayas_validas += 1
                    # Verde = Raya válida dentro de la zona
                    cv2.rectangle(out_img, (x, y), (x + w_rect, y + h_rect), (0, 255, 0), 3)
                else:
                    # Rojo = Basurita pequeña dentro de la zona
                    cv2.rectangle(out_img, (x, y), (x + w_rect, y + h_rect), (0, 0, 255), 1)

        # --- 4. CONTROL DE MOTORES ---
        cmd = MotorCommand()
        cmd.dir_servo = 1500 # Siempre derecho
        cmd.turn_signals = 0

        if not modo_activo:
            cmd.dir_dc, cmd.speed_dc = 0, 0
            cv2.putText(out_img, "MODO SEGURO (Apagado en RQT)", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            cmd.dir_dc = 1
            # Si contamos 3 o más rayas verdes dentro del cuadro amarillo -> FRENAR
            if rayas_validas >= min_rayas:
                cv2.putText(out_img, f"¡CEBRA DETECTADA! FRENANDO", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                cmd.speed_dc = 0
                cmd.stop_lights = 1 
            else:
                cv2.putText(out_img, f"Avanzando. Rayas en zona: {rayas_validas}", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cmd.speed_dc = vel_prueba
                cmd.stop_lights = 0

        self.pub_motor.publish(cmd)
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(out_img))

def main(args=None):
    rclpy.init(args=args)
    node = PruebaCruceNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        stop = MotorCommand()
        stop.dir_dc, stop.speed_dc, stop.dir_servo = 0, 0, 1500
        node.pub_motor.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()