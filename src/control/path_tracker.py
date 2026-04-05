""" Pure Pursuit path tracker """
import math

from util.math_utils import wrap_pi
from util.types import DriveCommand

# racecar geometry
WHEELBASE_M = 0.32
MAX_STEER_RAD = 0.35  # 20 degrees


class PurePursuitTracker:
    def __init__(
        self,
        path,
        min_lookahead_m=0.4,
        max_lookahead_m=1.5,
        cruise_speed=1.0,
        curve_slowdown=0.7,
        curve_horizon_m=2.5,
        finish_radius_m=0.3,
    ):
        self._path = path
        self._la_min = min_lookahead_m
        self._la_max = max_lookahead_m
        self._cruise = cruise_speed
        self._slow = curve_slowdown
        self._horizon = curve_horizon_m
        self._finish_r = finish_radius_m
        self._last_idx = 0
        self._finished = False
        # distance between two path points (meters)
        self._ds = (
            math.hypot(path[1].x_m - path[0].x_m, path[1].y_m - path[0].y_m)
            if len(path) >= 2 else 0.1
        )

    @property
    def finished(self):
        return self._finished

    def update(self, x, y, yaw):
        if not self._path:
            return DriveCommand(0.0, 0.0)

        # stop near the last path point
        tail = self._path[-1]
        if math.hypot(tail.x_m - x, tail.y_m - y) < self._finish_r:
            self._finished = True
            return DriveCommand(0.0, 0.0)

        # look closer and slow down if sharp turn ahead
        nearest_i = self._nearest_idx(x, y)
        t = min(1.0, self._upcoming_curvature(nearest_i) * 3.0)
        lookahead = self._la_max - t * (self._la_max - self._la_min)
        speed = self._cruise * (1.0 - self._slow * t)

        # pick the target point ahead, steer toward it
        la_i = self._lookahead_idx(nearest_i, x, y, lookahead)
        target = self._path[la_i]
        dx, dy = target.x_m - x, target.y_m - y
        alpha = wrap_pi(math.atan2(dy, dx) - yaw)
        ld = math.hypot(dx, dy)
        steer_rad = math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), ld) if ld > 1e-3 else 0.0
        steer_cmd = max(-1.0, min(1.0, steer_rad / MAX_STEER_RAD))
        return DriveCommand(speed=speed, angle=steer_cmd)

    # find closest path point to car
    def _nearest_idx(self, x, y):
        lo = max(0, self._last_idx - 5)
        hi = min(len(self._path), self._last_idx + 200)
        best_i, best_d2 = self._last_idx, float("inf")
        for i in range(lo, hi):
            d2 = (self._path[i].x_m - x) ** 2 + (self._path[i].y_m - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        if best_d2 > 4.0:  # full scan if > 2m away
            best_d2 = float("inf")
            for i in range(len(self._path)):
                d2 = (self._path[i].x_m - x) ** 2 + (self._path[i].y_m - y) ** 2
                if d2 < best_d2:
                    best_d2, best_i = d2, i
        self._last_idx = best_i
        return best_i

    # first path point at least lookahead meters away from the car
    def _lookahead_idx(self, start, x, y, lookahead):
        L2 = lookahead * lookahead
        for i in range(start, len(self._path)):
            d2 = (self._path[i].x_m - x) ** 2 + (self._path[i].y_m - y) ** 2
            if d2 >= L2:
                return i
        return len(self._path) - 1

    # turn tighter for sharper turns
    def _local_curvature(self, i, k=10):
        j = min(i + k, len(self._path) - 1)
        dyaw = wrap_pi(self._path[j].yaw_rad - self._path[i].yaw_rad)
        ds = max((j - i) * self._ds, 1e-3)
        return abs(dyaw) / ds

    # slow down early
    def _upcoming_curvature(self, i):
        n = max(1, int(self._horizon / self._ds))
        end = min(i + n, len(self._path) - 1)
        max_k = 0.0
        for j in range(i, end, 5):
            max_k = max(max_k, self._local_curvature(j))
        return max_k
