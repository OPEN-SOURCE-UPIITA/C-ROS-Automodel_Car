#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePreprocessingNode(Node):
    def __init__(self):
        super().__init__('image_preprocessing_node')

        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image', 
            self.listener_callback,
            10)

        self.publisher_img = self.create_publisher(Image, '/imagen/imagen_mejorada', 10)

        self.br = CvBridge()

        self.declare_parameter('contraste_alpha', 15)
        self.declare_parameter('brillo_beta', -90)

        self.declare_parameter('erosion_kernel_size', 5) 
        self.declare_parameter('erosion_iterations', 0)

        self.declare_parameter('blur_kernel_size', 3)

    def listener_callback(self, data):
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        alpha = self.get_parameter('contraste_alpha').value
        beta = self.get_parameter('brillo_beta').value
        k_size_erosion = self.get_parameter('erosion_kernel_size').value
        iterations_erosion = self.get_parameter('erosion_iterations').value
        k_size_blur = self.get_parameter('blur_kernel_size').value

        if k_size_erosion % 2 == 0: k_size_erosion += 1
        if k_size_erosion < 1: k_size_erosion = 1
        
        if k_size_blur % 2 == 0: k_size_blur += 1
        if k_size_blur < 1: k_size_blur = 1

        if iterations_erosion < 0: iterations_erosion = 0

        img_corregida = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)

        kernel_erosion = np.ones((k_size_erosion, k_size_erosion), np.uint8)
        img_erosionada = cv2.erode(img_corregida, kernel_erosion, iterations=iterations_erosion)

        img_final = cv2.GaussianBlur(img_erosionada, (k_size_blur, k_size_blur), 0)

        msg_salida = self.br.cv2_to_imgmsg(img_final, encoding='bgr8')
        self.publisher_img.publish(msg_salida)

def main(args=None):
    rclpy.init(args=args)
    nodo = ImagePreprocessingNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()