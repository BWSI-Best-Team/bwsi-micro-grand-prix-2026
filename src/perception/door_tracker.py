import math


class DoorTracker:
    def __init__(
        self,
        door_center_xy,
        track_map,
        blade_radius_m=1.5,
        center_exclusion_m=0.12,
        min_blade_points=4,
    ):
        self.cx, self.cy = door_center_xy
        self.map = track_map
        self.blade_radius = blade_radius_m
        self.center_exclusion_m = center_exclusion_m
        self.min_blade_points = min_blade_points
        self._debug_blade_count = 0
        self._debug_angle = None
        self._blade_points_world = []  # for visualization
        self._debug_fit_error_m = None
        self._debug_confidence = 0.0

    # return door global rotation angle
    def estimate_angle(self, lidar_scan, car_x, car_y, car_yaw):
        if lidar_scan is None:
            return None

        N = len(lidar_scan)
        blade_points = []
        blade_points_local = []

        for i in range(N):
            dist_cm = lidar_scan[i]
            if dist_cm <= 0 or dist_cm > 10000 or not math.isfinite(dist_cm):
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
            radius_m = math.hypot(dx, dy)
            if radius_m > self.blade_radius or radius_m < self.center_exclusion_m:
                continue

            # Keep points that are likely on one of the door blades.
            blade_points.append((wx, wy))
            blade_points_local.append((dx, dy))

        self._blade_points_world = blade_points
        self._debug_blade_count = len(blade_points_local)
        self._debug_fit_error_m = None
        self._debug_confidence = 0.0

        if len(blade_points_local) < self.min_blade_points:
            self._debug_angle = None
            return None

        angle, fit_error_m = _fit_cross_angle(blade_points_local)
        self._debug_angle = angle
        self._debug_fit_error_m = fit_error_m
        self._debug_confidence = _fit_confidence(len(blade_points_local), fit_error_m)
        return angle

    def debug_str(self):
        if self._debug_angle is None:
            return f"door: no blades ({self._debug_blade_count} pts)"
        deg = math.degrees(self._debug_angle)
        fit_cm = 100.0 * (self._debug_fit_error_m or 0.0)
        return (
            f"door: {deg:.1f}deg ({self._debug_blade_count} pts, "
            f"fit={fit_cm:.1f}cm, conf={self._debug_confidence:.2f})"
        )
def _fit_cross_angle(points_xy):
    half_pi = math.pi / 2.0
    coarse_step = math.radians(1.0)
    fine_step = math.radians(0.25)
    final_step = math.radians(0.05)

    best_angle = 0.0
    best_score = float("inf")

    angle = 0.0
    while angle < half_pi:
        score = _cross_fit_score(points_xy, angle)
        if score < best_score:
            best_score = score
            best_angle = angle
        angle += coarse_step

    best_angle, best_score = _refine_cross_angle(
        points_xy,
        best_angle,
        window_rad=math.radians(2.0),
        step_rad=fine_step,
    )
    best_angle, best_score = _refine_cross_angle(
        points_xy,
        best_angle,
        window_rad=math.radians(0.5),
        step_rad=final_step,
    )
    return best_angle, best_score


def _refine_cross_angle(points_xy, center_angle_rad, window_rad, step_rad):
    half_pi = math.pi / 2.0
    best_angle = center_angle_rad % half_pi
    best_score = _cross_fit_score(points_xy, best_angle)

    start = center_angle_rad - window_rad
    stop = center_angle_rad + window_rad
    steps = max(1, int(round((stop - start) / step_rad)))
    for i in range(steps + 1):
        angle = (start + i * step_rad) % half_pi
        score = _cross_fit_score(points_xy, angle)
        if score < best_score:
            best_score = score
            best_angle = angle
    return best_angle, best_score


def _cross_fit_score(points_xy, angle_rad):
    errors_m = []
    axis_a = angle_rad
    axis_b = angle_rad + math.pi / 2.0

    sin_a = math.sin(axis_a)
    cos_a = math.cos(axis_a)
    sin_b = math.sin(axis_b)
    cos_b = math.cos(axis_b)

    for x_m, y_m in points_xy:
        # Perpendicular distance to the nearer blade axis through the center.
        dist_a = abs(-sin_a * x_m + cos_a * y_m)
        dist_b = abs(-sin_b * x_m + cos_b * y_m)
        errors_m.append(min(dist_a, dist_b))

    if not errors_m:
        return float("inf")

    errors_m.sort()
    keep_count = max(1, int(math.ceil(len(errors_m) * 0.75)))
    kept = errors_m[:keep_count]
    return sum(kept) / len(kept)


def _fit_confidence(point_count, fit_error_m):
    count_term = min(1.0, point_count / 12.0)
    fit_term = max(0.0, 1.0 - fit_error_m / 0.20)
    return count_term * fit_term
