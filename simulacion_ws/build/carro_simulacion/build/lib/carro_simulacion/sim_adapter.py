import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor_msgs.msg import MotorCommand

class SimAdapter(Node):
    def __init__(self):
        super().__init__('sim_adapter')
        
        # Publicador hacia Gazebo
        self.pub = self.create_publisher(Twist, '/model/carro/cmd_vel', 10)

        # Suscriptor único a tu control
        self.create_subscription(MotorCommand, '/motor_command', self.cmd_cb, 10)
        self.get_logger().info("Adaptador iniciado: Traduciendo /motor_command a Twist")

    def cmd_cb(self, msg):
        t = Twist()
        
        # 1. Velocidad Lineal
        speed = msg.speed_dc / 100.0
        t.linear.x = speed if msg.dir_dc == 1 else -speed if msg.dir_dc == 2 else 0.0
        
        # 2. Velocidad Angular (con zona muerta y conversión matemática)
        if 1480 <= msg.dir_servo <= 1520:
            t.angular.z = 0.0
        else:
            t.angular.z = (msg.dir_servo - 1500) * 0.0025

        self.pub.publish(t)

def main():
    rclpy.init()
    rclpy.spin(SimAdapter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()