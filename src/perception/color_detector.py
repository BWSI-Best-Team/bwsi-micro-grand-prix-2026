import cv2 as cv
import numpy as np
import racecar_utils as rc_utils


# HSV color ranges
BLUE = ((90, 200, 200), (120, 255, 255))
GREEN = ((35, 50, 50), (75, 255, 255))
RED = ((0, 150, 100), (10, 255, 255))
ORANGE = ((10, 50, 50), (20, 255, 255))
YELLOW = ((20, 50, 50), (30, 255, 255))
PURPLE = ((140, 50, 50), (160, 255, 255))

MIN_CONTOUR_AREA = 1000


class ColorDetector:
    def __init__(self):
        self.contour_center = None
        self.contour_area = 0
        self.detected_color = ""

    def update(self, color_image, colors=None, crop_window=None):
        self.contour_center = None
        self.contour_area = 0
        self.detected_color = ""

        if color_image is None:
            return

        image = color_image
        if crop_window is not None:
            image = rc_utils.crop(image, crop_window[0], crop_window[1])

        if colors is None:
            colors = [
                ("RED", RED), ("GREEN", GREEN), ("BLUE", BLUE),
                ("ORANGE", ORANGE), ("YELLOW", YELLOW), ("PURPLE", PURPLE),
            ]

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        best_contour = None
        best_area = 0
        best_color = ""

        for name, (hsv_min, hsv_max) in colors:
            mask = cv.inRange(hsv, hsv_min, hsv_max)
            contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv.contourArea(contour)
                if area > MIN_CONTOUR_AREA and area > best_area:
                    best_contour = contour
                    best_area = area
                    best_color = name

        if best_contour is not None:
            self.contour_center = rc_utils.get_contour_center(best_contour)
            self.contour_area = best_area
            self.detected_color = best_color

    def debug_str(self):
        if self.contour_center is None:
            return "color: none"
        return f"color: {self.detected_color} area={self.contour_area} at {self.contour_center}"
