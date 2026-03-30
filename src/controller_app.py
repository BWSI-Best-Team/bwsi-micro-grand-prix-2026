from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any

import racecar_utils as rc_utils
from controller_config import ControllerConfig, load_controller_config


@dataclass(slots=True)
class SensorState:
    color_image: Any = None
    depth_image: Any = None
    lidar_scan: Any = None
    front_distance_cm: float = 0.0


@dataclass(slots=True)
class DriveCommand:
    speed: float = 0.0
    angle: float = 0.0


class RacecarState(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    RUNNING = auto()


class GrandPrixController:
    def __init__(self, rc: Any) -> None:
        self._rc = rc
        self._config: ControllerConfig = load_controller_config()
        self._frame_count = 0
        self._mode = RacecarState.IDLE
        self._speed = 0.0
        self._angle = 0.0
        self._sensors = SensorState()

    def start(self) -> None:
        self._frame_count = 0
        self._mode = RacecarState.RUNNING
        self._speed = 0.0
        self._angle = 0.0
        self._sensors = SensorState()
        self._rc.drive.stop()

        print(
            ">> BWSI 2026 Grand Prix Best Team controller\n"
        )

    def update(self) -> None:
        self._rc.drive.set_max_speed(1.0) # Located here per regulation

        self._frame_count += 1
        self._sample_sensors()
        self._update_mode()

        command = self._compute_command()
        self._speed = command.speed
        self._angle = command.angle
        self._rc.drive.set_speed_angle(self._speed, self._angle)

    def update_slow(self) -> None: # For logging only
        if not self._config.print_log:
            return

        print(
            "[status] "
            f"frame={self._frame_count} "
            f"mode={self._mode.name} "
            f"cmd=({self._speed:.2f},{self._angle:.2f}) "
            f"front={self._sensors.front_distance_cm:.1f}cm "
            f"color={'yes' if self._sensors.color_image is not None else 'no'}"
        )

    def _sample_sensors(self) -> None:
        self._sensors.color_image = self._rc.camera.get_color_image()
        self._sensors.lidar_scan = self._rc.lidar.get_samples()

        if self._sensors.lidar_scan is None:
            self._sensors.front_distance_cm = 0.0
            return

        self._sensors.front_distance_cm = rc_utils.get_lidar_average_distance(
            self._sensors.lidar_scan, 0.0
        )

    def _update_mode(self) -> None:
        if self._sensors.color_image is None:
            self._mode = RacecarState.INITIALIZING
        else:
            self._mode = RacecarState.RUNNING

    def _compute_command(self) -> DriveCommand:
        # TODO: add real controller when perception and policy are ready.
        return DriveCommand(speed=1.0, angle=0.0)
