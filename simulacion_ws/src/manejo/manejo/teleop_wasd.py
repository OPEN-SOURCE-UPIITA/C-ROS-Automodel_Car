import select
import sys
import termios
import time
import tty

import rclpy
from motor_msgs.msg import MotorCommand
from rclpy.node import Node


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.declare_parameter('speed', 50)
        self.declare_parameter('speed_step', 5)
        self.declare_parameter('input_timeout', 0.5)

        self.speed = max(0, min(100, self.get_parameter('speed').value))
        self.speed_step = max(1, self.get_parameter('speed_step').value)
        self.input_timeout = max(0.1, self.get_parameter('input_timeout').value)
        self.direction = 0
        self.steering = 1500
        self.last_input = time.monotonic()

        self.publisher = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.create_timer(0.05, self.publish_command)
        self.get_logger().info(
            'WASD: W avanzar, S reversa, A/D girar, espacio detener; +/- velocidad'
        )

    def read_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def publish_command(self):
        key = self.read_key()
        if key is not None:
            self.handle_key(key)

        if time.monotonic() - self.last_input > self.input_timeout:
            self.direction = 0
            self.steering = 1500

        message = MotorCommand()
        message.dir_dc = self.direction
        message.speed_dc = self.speed if self.direction else 0
        message.dir_servo = self.steering
        self.publisher.publish(message)

    def handle_key(self, key):
        if key == 'w':
            self.direction = 1
        elif key == 's':
            self.direction = 2
        elif key == 'a':
            self.steering = 1740
        elif key == 'd':
            self.steering = 1110
        elif key == ' ':
            self.direction = 0
            self.steering = 1500
        elif key in ('+', '='):
            self.speed = min(100, self.speed + self.speed_step)
        elif key == '-':
            self.speed = max(0, self.speed - self.speed_step)
        else:
            return

        self.last_input = time.monotonic()
        self.get_logger().info(
            f'Velocidad: {self.speed}% | direccion: {self.direction} | servo: {self.steering}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error('Ejecuta este nodo desde una terminal interactiva.')
        node.destroy_node()
        rclpy.shutdown()
        return

    terminal_fd = sys.stdin.fileno()
    terminal_settings = termios.tcgetattr(terminal_fd)
    try:
        tty.setcbreak(terminal_fd)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = MotorCommand()
        stop.dir_dc = 0
        stop.speed_dc = 0
        stop.dir_servo = 1500
        node.publisher.publish(stop)
        termios.tcsetattr(terminal_fd, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        rclpy.shutdown()