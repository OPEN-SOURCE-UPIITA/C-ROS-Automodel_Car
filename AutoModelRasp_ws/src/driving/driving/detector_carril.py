#!/usr/bin/env python3
import rclpy
from driving.vision_utils import segmentacion_color
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorCarrilNode(Node):
    def __init__(self):
        super().__init__('detector_carril')

        # Parámetros
        self.declare_parameter('umbral_blanco', 110)
        self.declare_parameter('corte_y_sup_pct', 80)
        self.declare_parameter('corte_y_inf_pct', 5)
        self.declare_parameter('ancho_top', 180)
        self.declare_parameter('ancho_bot', 550)
        self.declare_parameter('suavizado_pct', 50)
        self.declare_parameter('min_puntos', 50)
        self.declare_parameter('min_pendiente', 0.2)
        self.declare_parameter('base_offset_l', 110)
        self.declare_parameter('base_offset_r', 140)
        self.declare_parameter('activar_seguimiento', True)
        self.declare_parameter('confianza_un_carril', 0.76)
        self.declare_parameter('usar_anclaje_base', True)
        self.declare_parameter('max_curvatura', 0.1)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.bridge = CvBridge()
        self.last_error = 0.0
        self.left_fit = None
        self.right_fit = None
        self.lane_width_pixels = 350.0

        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            10)

        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def procesar_puntos_carril(self, mask, cx, y_inf, anchors):
        ys, xs = np.where(mask > 0)
        l_fit, r_fit = None, None

        if len(xs) > 0:
            points = np.column_stack((xs, ys))
            left_pts = points[points[:, 0] < cx]
            right_pts = points[points[:, 0] > cx]

            l_fit = self.fit_lane_logic(left_pts, anchors[0], y_inf)
            r_fit = self.fit_lane_logic(right_pts, anchors[1], y_inf)

        return l_fit, r_fit

    def fit_lane_logic(self, points, anchor_x, h):
        if len(points) < self.get_parameter('min_puntos').value:
            return None

        xs, ys = points[:, 0], points[:, 1]

        if self.get_parameter('usar_anclaje_base').value:
            xs = np.concatenate((xs, np.full(25, anchor_x)))
            ys = np.concatenate((ys, np.full(25, h)))

        try:
            coeffs = np.polyfit(ys, xs, 2)

            max_curv = self.get_parameter('max_curvatura').value
            if abs(coeffs[0]) > max_curv:
                lin = np.polyfit(ys, xs, 1)
                coeffs = np.array([0.0, lin[0], lin[1]])

            min_slope = self.get_parameter('min_pendiente').value
            if abs(2 * coeffs[0] * h + coeffs[1]) > (1.0 / min_slope):
                return None

            return coeffs
        except:
            return None

    def render_poly_restricted(self, img, coeffs, color, y_min, y_max, side):
        cx = img.shape[1] // 2
        py = np.linspace(y_min, y_max, 25)
        px = coeffs[0]*py**2 + coeffs[1]*py + coeffs[2]

        if side == 'L':
            px = np.minimum(px, cx - 2)
        else:
            px = np.maximum(px, cx + 2)

        pts = np.array([np.transpose(np.vstack([px, py]))], np.int32)
        cv2.polylines(img, pts, False, color, 4)

    def dibujo_virtual(self, img, polygon, l_fit, r_fit, target_x, anchors, y_lims):
        y_sup, y_inf = y_lims
        cx = img.shape[1] // 2

        cv2.polylines(img, polygon, True, (255, 0, 0), 2)
        cv2.line(img, (cx, y_inf), (cx, y_sup), (255, 255, 0), 1)

        if self.get_parameter('usar_anclaje_base').value:
            cv2.circle(img, (anchors[0], y_inf), 7, (255, 0, 255), -1)
            cv2.circle(img, (anchors[1], y_inf), 7, (255, 0, 255), -1)

        if l_fit is not None:
            self.render_poly_restricted(img, l_fit, (0, 255, 0), y_sup, y_inf, 'L')

        if r_fit is not None:
            self.render_poly_restricted(img, r_fit, (0, 255, 0), y_sup, y_inf, 'R')

        if target_x is not None:
            cv2.circle(img, (int(target_x), y_sup), 12, (0, 255, 255), -1)
            cv2.line(img, (cx, y_sup), (int(target_x), y_sup), (0, 255, 255), 2)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        cx = w // 2
        debug = frame.copy()

        # ROI
        y_sup = int(h * (self.get_parameter('corte_y_sup_pct').value / 100.0))
        y_inf = h - int(h * (self.get_parameter('corte_y_inf_pct').value / 100.0))
        w_t = self.get_parameter('ancho_top').value
        w_b = self.get_parameter('ancho_bot').value

        polygon = np.array([[
            (cx - w_b//2, y_inf),
            (cx + w_b//2, y_inf),
            (cx + w_t, y_sup),
            (cx - w_t, y_sup)
        ]], np.int32)

        # SEGMENTACIÓN
        umbral = self.get_parameter('umbral_blanco').value
        mask = segmentacion_color(frame, polygon, umbral)

        # ANCHORS (ANTES de usarlos)
        off_l = self.get_parameter('base_offset_l').value
        off_r = self.get_parameter('base_offset_r').value
        anchors = (cx - off_l, cx + off_r)

        l_fit_raw, r_fit_raw = self.procesar_puntos_carril(mask, cx, y_inf, anchors)

        # SUAVIZADO
        alpha = self.get_parameter('suavizado_pct').value / 100.0

        if l_fit_raw is not None:
            self.left_fit = l_fit_raw if self.left_fit is None else alpha*l_fit_raw + (1-alpha)*self.left_fit
        else:
            self.left_fit = None

        if r_fit_raw is not None:
            self.right_fit = r_fit_raw if self.right_fit is None else alpha*r_fit_raw + (1-alpha)*self.right_fit
        else:
            self.right_fit = None

        # TARGET
        tx_l = np.polyval(self.left_fit, y_sup) if self.left_fit is not None else None
        tx_r = np.polyval(self.right_fit, y_sup) if self.right_fit is not None else None

        target_x = cx
        confianza = 1.0

        if tx_l is not None and tx_r is not None:
            target_x = (tx_l + tx_r) / 2
            self.lane_width_pixels = 0.95*self.lane_width_pixels + 0.05*(tx_r - tx_l)
        elif tx_l is not None:
            target_x = tx_l + self.lane_width_pixels/2
            confianza = self.get_parameter('confianza_un_carril').value
        elif tx_r is not None:
            target_x = tx_r - self.lane_width_pixels/2
            confianza = self.get_parameter('confianza_un_carril').value

        error = np.clip(((target_x - cx) / cx) * confianza, -1.0, 1.0)
        self.last_error = 0.7*self.last_error + 0.3*error

        if self.get_parameter('activar_seguimiento').value:
            self.publisher_error.publish(Float32(data=float(self.last_error)))

        self.dibujo_virtual(debug, polygon, self.left_fit, self.right_fit, target_x, anchors, (y_sup, y_inf))

        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(mask))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(debug))


def main(args=None):
    rclpy.init(args=args)
    node = DetectorCarrilNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
