#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        self.declare_parameter('apertura_general', 30.0) 
        
        # 🏴‍☠️ LA CÁPSULA DEL POSTE: Solo borramos este pequeñísimo rango exacto
        self.declare_parameter('poste_min', 0.065) # 6.5 cm
        self.declare_parameter('poste_max', 0.085) # 8.5 cm
        
        self.declare_parameter('dist_max_izq', 1.20) 
        self.declare_parameter('dist_max_der', 1.50)
        self.declare_parameter('dist_max_atr', 1.00)
        
        self.get_logger().info("✅ Filtro de Cápsula iniciado. Poste asilado matemáticamente.")

    def scan_callback(self, msg):
        apertura = self.get_parameter('apertura_general').value
        p_min = self.get_parameter('poste_min').value
        p_max = self.get_parameter('poste_max').value
        
        d_max_izq = self.get_parameter('dist_max_izq').value
        d_max_der = self.get_parameter('dist_max_der').value
        d_max_atr = self.get_parameter('dist_max_atr').value

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min, filtered_msg.angle_max = msg.angle_min, msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.range_min, filtered_msg.range_max = msg.range_min, msg.range_max
        
        new_ranges = list(msg.ranges)
        
        for i in range(len(new_ranges)):
            d = new_ranges[i]
            if math.isinf(d) or math.isnan(d) or d < 0.02:
                continue

            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment) % 360.0
            keep_point = False
            
            # 1. IZQUIERDA
            diff_izq = abs((angle_deg - 90.0 + 180) % 360 - 180)
            if diff_izq <= apertura and d <= d_max_izq: keep_point = True
                
            # 2. ATRÁS
            diff_atr = abs((angle_deg - 180.0 + 180) % 360 - 180)
            if diff_atr <= apertura and d <= d_max_atr: keep_point = True
                
            # 3. DERECHA
            diff_der = abs((angle_deg - 270.0 + 180) % 360 - 180)
            if diff_der <= apertura and d <= d_max_der:
                keep_point = True
            
            # 🔥 MAGIA: Si el punto sobrevivió, pero está EXACTAMENTE donde vive el poste, lo matamos.
            if keep_point and (p_min <= d <= p_max):
                keep_point = False

            if not keep_point:
                new_ranges[i] = float('inf')

        filtered_msg.ranges = new_ranges
        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args); rclpy.spin(LidarFilterNode()); rclpy.shutdown()

if __name__ == '__main__': main()