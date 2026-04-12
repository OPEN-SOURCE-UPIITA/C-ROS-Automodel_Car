import cv2
import numpy as np

def segmentacion_color(frame, polygon, umbral):
    h, w = frame.shape[:2]
    mask_roi = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask_roi, polygon, 255)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask_blancos = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY)

    return cv2.bitwise_and(mask_blancos, mask_roi)
