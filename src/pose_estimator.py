from __future__ import annotations

from dataclasses import dataclass
import math

@dataclass(slots=True)
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    x_vel: float = 0.0
    y_vel: float = 0.0


class PoseEstimator:
    def __init__(self):
        self._pose = Pose2D()

    @property
    def pose(self) -> Pose2D:
        return self._pose
    
    def reset(self) -> None:
        self._pose = Pose2D()

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
