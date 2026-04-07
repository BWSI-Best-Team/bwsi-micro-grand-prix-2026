import numpy as np
import racecar_utils as rc_utils


class DepthDetector:
    def __init__(self):
        self.center_distance_cm = 0.0
        self.closest_pixel = None
        self.closest_distance_cm = 0.0

    def update(self, depth_image, crop_window=None):
        self.center_distance_cm = 0.0
        self.closest_pixel = None
        self.closest_distance_cm = 0.0

        if depth_image is None:
            return

        image = depth_image
        if crop_window is not None:
            image = rc_utils.crop(image, crop_window[0], crop_window[1])

        self.center_distance_cm = rc_utils.get_depth_image_center_distance(image)
        self.closest_pixel = rc_utils.get_closest_pixel(image)
        if self.closest_pixel is not None:
            self.closest_distance_cm = image[self.closest_pixel[0], self.closest_pixel[1]]

    def debug_str(self):
        return f"depth: center={self.center_distance_cm:.1f}cm closest={self.closest_distance_cm:.1f}cm"
