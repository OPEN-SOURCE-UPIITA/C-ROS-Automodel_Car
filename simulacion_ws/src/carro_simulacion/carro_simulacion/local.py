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
        
        # CORRECCIÓN: Se agregó el guion bajo para que coincida en todo el código
        self.publisher_ = self.create_publisher(MotorCommand, '/motor_command', 10)

        # Configuración del Socket
        self.server_ip = '127.0.0.1' # Es más seguro usar 127.0.0.1 que 'localhost' en ROS 2
        self.server_port = 65432
        
        # Iniciamos el cliente de red en un hilo separado para no bloquear ROS 2
        self.red_thread = threading.Thread(target=self.conectar_y_escuchar, daemon=True)
        self.red_thread.start()

    def conectar_y_escuchar(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Conectando al servidor {self.server_ip}:{self.server_port}...')
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((self.server_ip, self.server_port))
                    
                    # Nos identificamos ante el servidor
                    s.sendall(b"SOY_EL_ROBOT\n")
                    self.get_logger().info('¡Conectado! Esperando comandos...')
                    
                    buffer = ""
                    while rclpy.ok():
                        data = s.recv(1024)
                        if not data:
                            self.get_logger().warn('El servidor cerró la conexión.')
                            break
                        
                        # Decodificamos y separamos por saltos de línea (por si llegan muy rápido)
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
            
            ros_msg = MotorCommand()
            ros_msg.dir_dc = data.get('dir_dc', 0)
            ros_msg.speed_dc = data.get('speed_dc', 0)
            ros_msg.dir_servo = data.get('dir_servo', 1500)
            ros_msg.stop_lights = data.get('stop_lights', 0)
            ros_msg.turn_signals = data.get('turn_signals', 0)

            # Ahora esta línea funcionará perfectamente
            self.publisher_.publish(ros_msg)
            
            # Descomentado para ver en tiempo real lo que recibe el nodo
            self.get_logger().info(f'Comando aplicado: {data}')
            
        except json.JSONDecodeError:
            self.get_logger().error('Se recibió un comando con formato dañado.')

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