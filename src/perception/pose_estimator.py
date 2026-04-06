from __future__ import annotations

from dataclasses import dataclass
import math

@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    x_vel: float = 0.0
    y_vel: float = 0.0


class PoseEstimator:
    def __init__(self):
        self._pose = Pose2D()
        self._last_x = None
        self._last_y = None
        self._speed_mps = 0.0

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    def speed_mps(self) -> float:
        return self._speed_mps

    def reset(self) -> None:
        self._pose = Pose2D()
        self._last_x = None
        self._last_y = None
        self._speed_mps = 0.0

    def update_speed_from_position(self, x_m, y_m, dt):
        if self._last_x is not None and dt > 0:
            dx = x_m - self._last_x
            dy = y_m - self._last_y
            self._speed_mps = math.hypot(dx, dy) / dt
        self._last_x = x_m
        self._last_y = y_m

    def update(self, x_accel: float, y_accel: float, yaw_accel: float, dt: float = 0.0) -> None:
        self._pose.yaw_rad += yaw_accel * dt

        # Normalize to world space
        x_accel_norm = x_accel * math.cos(self._pose.yaw_rad) - y_accel * math.sin(self._pose.yaw_rad)
        y_accel_norm = x_accel * math.sin(self._pose.yaw_rad) + y_accel * math.cos(self._pose.yaw_rad)

        # Double numerical integration
        self._pose.x_m += self._pose.x_vel * dt + 0.5 * x_accel_norm * dt * dt
        self._pose.y_m += self._pose.y_vel * dt + 0.5 * y_accel_norm * dt * dt
        
        self._pose.x_vel += x_accel_norm * dt
        self._pose.y_vel += y_accel_norm * dt
