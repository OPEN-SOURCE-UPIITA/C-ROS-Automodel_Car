import rclpy
from rclpy.node import Node
import socket
import threading
import json
import time

from motor_msgs.msg import MotorCommand

class SocketToRos2Bridge(Node):
    def __init__(self):
        super().__init__('socket_to_ros2_bridge')
        
        # Publicador
        self.publisher_ = self.create_publisher(MotorCommand, '/motor_command', 10)

        # --- NUEVO: Memoria del último comando recibido ---
        self.current_cmd = MotorCommand()
        self.current_cmd.dir_servo = 1500

        # --- NUEVO: Timer que publica a 10 Hz (cada 0.1 segundos) ---
        self.timer = self.create_timer(0.1, self.publicar_constante)

        # Configuración del Socket
        self.server_ip = '16.59.195.73' 
        self.server_port = 6000
        
        # Iniciamos el cliente de red en un hilo separado
        self.red_thread = threading.Thread(target=self.conectar_y_escuchar, daemon=True)
        self.red_thread.start()

    def conectar_y_escuchar(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Conectando al servidor {self.server_ip}:{self.server_port}...')
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((self.server_ip, self.server_port))
                    
                    s.sendall(b"SOY_EL_ROBOT\n")
                    self.get_logger().info('¡Conectado! Esperando comandos...')
                    
                    buffer = ""
                    while rclpy.ok():
                        data = s.recv(1024)
                        if not data:
                            self.get_logger().warn('El servidor cerró la conexión.')
                            break
                        
                        buffer += data.decode('utf-8')
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            if line.strip():
                                self.procesar_comando(line)
                                
            except Exception as e:
                self.get_logger().error(f'Error de conexión: {e}. Reintentando en 3 segundos...')
                time.sleep(3)

    def procesar_comando(self, json_str):
        try:
            data = json.loads(json_str)
            
            # --- MODIFICADO: Solo guardamos los datos en la memoria interna ---
            self.current_cmd.dir_dc = data.get('dir_dc', 0)
            self.current_cmd.speed_dc = data.get('speed_dc', 0)
            self.current_cmd.dir_servo = data.get('dir_servo', 1500)
            self.current_cmd.stop_lights = data.get('stop_lights', 0)
            self.current_cmd.turn_signals = data.get('turn_signals', 0)
            
        except json.JSONDecodeError:
            self.get_logger().error('Se recibió un comando con formato dañado.')

    # --- NUEVO: Función que ejecuta el Timer automáticamente ---
    def publicar_constante(self):
        # Publica el estado actual continuamente hacia los motores
        self.publisher_.publish(self.current_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SocketToRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()