import math
import numpy as np


class DoorTracker:
    def __init__(self, door_center_xy, track_map, blade_radius_m=1.5):
        self.cx, self.cy = door_center_xy
        self.map = track_map
        self.blade_radius = blade_radius_m
        self._debug_blade_count = 0
        self._debug_angle = None
        self._blade_points_world = []  # for visualization

    # return door global rotation angle
    def estimate_angle(self, lidar_scan, car_x, car_y, car_yaw):
        if lidar_scan is None:
            return None

        N = len(lidar_scan)
        blade_angles = []
        blade_points = []

        for i in range(N):
            dist_cm = lidar_scan[i]
            if dist_cm <= 0:
                continue

            # LiDAR point -> world coordinates
            lidar_angle = i * 2.0 * math.pi / N  # 0 front, clockwise
            world_angle = car_yaw - lidar_angle   # convert to world frame, Counter Clockwise
            dist_m = dist_cm / 100.0
            wx = car_x + dist_m * math.cos(world_angle)
            wy = car_y + dist_m * math.sin(world_angle)

            # check static map that if the point is moving or not matchiing
            row, col = self.map.world_to_grid(wx, wy)
            if not self.map.in_bounds(row, col):
                continue
            if self.map.grid[row, col] == 255:
                continue  # wall

            # check wheather is dynamic object by matching the map
            # and only keep points near door center
            dx = wx - self.cx
            dy = wy - self.cy
            if math.hypot(dx, dy) > self.blade_radius:
                continue

            # ckecked it's on a door blade
            blade_angles.append(math.atan2(dy, dx))
            blade_points.append((wx, wy))

        self._blade_points_world = blade_points
        self._debug_blade_count = len(blade_angles)

        if not blade_angles:
            self._debug_angle = None
            return None

        angle = _cluster_mean_mod90(blade_angles)
        self._debug_angle = angle
        return angle

    def debug_str(self):
        if self._debug_angle is None:
            return f"door: no blades ({self._debug_blade_count} pts)"
        deg = math.degrees(self._debug_angle)
        return f"door: {deg:.1f}deg ({self._debug_blade_count} pts)"


def _cluster_mean_mod90(angles):
    half_pi = math.pi / 2.0
    # map to [0, pi/2)
    mapped = [(a % half_pi) for a in angles]

    # average angles on a circle so 2 and 88 degree goes to 0
    sin_sum = sum(math.sin(2.0 * a) for a in mapped)
    cos_sum = sum(math.cos(2.0 * a) for a in mapped)
    mean_2a = math.atan2(sin_sum, cos_sum)
    mean = mean_2a / 2.0

    # normalize to [0, pi/2)
    while mean < 0:
        mean += half_pi
    while mean >= half_pi:
        mean -= half_pi

    return mean
