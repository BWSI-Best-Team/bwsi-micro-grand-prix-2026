"""
Shared data types
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class DriveCommand:
    # [-1, 1]
    speed: float
    angle: float
