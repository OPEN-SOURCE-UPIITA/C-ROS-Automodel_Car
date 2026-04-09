#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorCarrilNode(Node):
    def __init__(self):
        super().__init__('detector_carril')

        # PARAMS
        self.declare_parameter('umbral_blanco', 180)
        self.declare_parameter('corte_y_pct', 55)
        self.declare_parameter('ancho_top', 120)
        self.declare_parameter('ancho_bot', 600)
        self.declare_parameter('suavizado_pct', 20)
        self.declare_parameter('ancho_carril_px', 350)
        self.declare_parameter('max_dist_outlier', 50)
        self.declare_parameter('min_verticalidad_vy', 50)
        self.declare_parameter('activar_seguimiento', False)

        self.declare_parameter('base_left_ratio', 25.0)
        self.declare_parameter('base_right_ratio', 25.0)
        self.declare_parameter('top_left_ratio', 25.0)
        self.declare_parameter('top_right_ratio', 25.0)

        self.declare_parameter('max_dif_angulo_deg', 10.0)
        self.declare_parameter('max_dif_pos_px', 80)

        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            10)

        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.pub_mascara = self.create_publisher(CompressedImage, '/vision/carril_mascara/compressed', 10)
        self.pub_resultado = self.create_publisher(CompressedImage, '/vision/carril_resultado/compressed', 10)

        self.bridge = CvBridge()

        self.last_error = 0.0
        self.left_lane = None
        self.right_lane = None

    # ---------------- UTILIDADES ----------------

    def lane_angle(self, lane):
        vx, vy, _, _ = lane
        return np.degrees(np.arctan2(vy, vx))

    def fit_lane(self, points, min_vy):
        if len(points) < 50:
            return None
        line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = line.flatten()
        if abs(vy) < min_vy:
            return None
        return line.flatten()

    def get_top_point(self, points, corte_y):
        if len(points) < 10:
            return None
        top_pts = points[points[:,1] < corte_y + 40]
        if len(top_pts) == 0:
            return None
        return int(np.mean(top_pts[:,0]))

    def fallback_lane(self, x_base, x_top, y_base, y_top):
        vx = x_top - x_base
        vy = y_top - y_base
        return np.array([vx, vy, x_base, y_base], dtype=np.float32)

    def draw_lane(self, img, lane, color):
        vx, vy, x0, y0 = lane
        h = img.shape[0]

        y1, y2 = int(h * 0.4), h
        divisor = vy if abs(vy) > 1e-3 else 1e-3

        x1 = int(x0 + (y1 - y0) * vx / divisor)
        x2 = int(x0 + (y2 - y0) * vx / divisor)

        cv2.line(img, (x1, y1), (x2, y2), color, 5)
        return x2

    # ---------------- CALLBACK ----------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        frame = cv2.resize(frame, (640, 480))
        h, w, _ = frame.shape
        debug = frame.copy()

        # ---------------- PARÁMETROS ----------------
        activar = self.get_parameter('activar_seguimiento').value
        umbral = self.get_parameter('umbral_blanco').value
        alpha = self.get_parameter('suavizado_pct').value / 100.0
        min_vy = self.get_parameter('min_verticalidad_vy').value / 100.0

        base_left_ratio = self.get_parameter('base_left_ratio').value / 100.0
        base_right_ratio = self.get_parameter('base_right_ratio').value / 100.0
        top_left_ratio = self.get_parameter('top_left_ratio').value / 100.0
        top_right_ratio = self.get_parameter('top_right_ratio').value /100.0

        corte_y = int(h * (self.get_parameter('corte_y_pct').value / 100.0))
        cx = w // 2
        w_t = self.get_parameter('ancho_top').value
        w_b = self.get_parameter('ancho_bot').value

        # ---------------- POLÍGONO ----------------
        polygon = np.array([[
            (cx - w_b//2, h),
            (cx + w_b//2, h),
            (cx + w_t, corte_y),
            (cx - w_t, corte_y)
        ]], np.int32)

        cv2.polylines(debug, polygon, True, (255, 0, 0), 2)

        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)

        # ---------------- SEGMENTACIÓN ----------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask_blancos = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask_blancos, mask_roi)

        ys, xs = np.where(mask > 0)

        if len(xs) > 0:
            points = np.column_stack((xs, ys))

            # -------- DIVISIÓN IZQ / DER --------
            left_region = points[points[:, 0] < cx]
            right_region = points[points[:, 0] >= cx]

            # -------- PUNTOS FIJOS --------
            x_base_left = cx - int(w_b * base_left_ratio)
            x_base_right = cx + int(w_b * base_right_ratio)

            y_base = h - 5
            y_top = corte_y

            # DEBUG puntos base
            cv2.circle(debug, (x_base_left, y_base), 8, (255, 0, 255), -1)
            cv2.circle(debug, (x_base_right, y_base), 8, (255, 0, 255), -1)

            # -------- FIT NORMAL --------
            left_fit = self.fit_lane(left_region, min_vy)
            right_fit = self.fit_lane(right_region, min_vy)

            # -------- TOP POINTS --------
            top_left = self.get_top_point(left_region, corte_y)
            top_right = self.get_top_point(right_region, corte_y)

            # -------- FALLBACK --------
            if left_fit is None:
                if top_left is None:
                    top_left = cx - int(w_t * top_left_ratio)
                left_fit = self.fallback_lane(x_base_left, top_left, y_base, y_top)

            if right_fit is None:
                if top_right is None:
                    top_right = cx + int(w_t * top_right_ratio)
                right_fit = self.fallback_lane(x_base_right, top_right, y_base, y_top)

            # -------- SUAVIZADO --------
            self.left_lane = left_fit if self.left_lane is None else alpha * left_fit + (1 - alpha) * self.left_lane
            self.right_lane = right_fit if self.right_lane is None else alpha * right_fit + (1 - alpha) * self.right_lane

            # -------- DIBUJO --------
            xl = self.draw_lane(debug, self.left_lane, (0, 255, 0))
            xr = self.draw_lane(debug, self.right_lane, (0, 255, 0))

            target_x = (xl + xr) // 2

            # -------- ERROR --------
            mid = w // 2
            error = np.clip((target_x - mid) / float(mid), -1.0, 1.0)
            self.last_error = 0.7 * self.last_error + 0.3 * error
            self.publisher_error.publish(Float32(data=float(self.last_error)))

            # -------- VISUAL --------
            cv2.circle(debug, (int(target_x), h - 20), 10, (255, 255, 0), -1)
            cv2.line(debug, (mid, h - 20), (int(target_x), h - 20), (0, 255, 255), 3)

            modo = "ESTRUCTURADO" if activar else "FLEXIBLE"
            cv2.putText(debug, f"MODO: {modo}", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # ---------------- PUBLICACIÓN ----------------
        self.pub_mascara.publish(self.bridge.cv2_to_compressed_imgmsg(mask))
        self.pub_resultado.publish(self.bridge.cv2_to_compressed_imgmsg(debug))


def main(args=None):
    rclpy.init(args=args)
    node = DetectorCarrilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
