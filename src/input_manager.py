from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import racecar_utils as rc_utils


@dataclass(slots=True)
class InputState:
    color_image: Any = None
    depth_image: Any = None
    lidar_scan: Any = None
    front_distance_cm: float = 0.0


class InputManager:
    def __init__(self, rc: Any) -> None:
        self._rc = rc
        self._state = InputState()

    @property
    def state(self) -> InputState:
        return self._state

    def reset(self) -> None:
        self._state = InputState()

    def update(self) -> None:
        self._state.color_image = self._rc.camera.get_color_image()
        self._state.depth_image = self._rc.camera.get_depth_image()
        self._state.lidar_scan = self._rc.lidar.get_samples()

        if self._state.lidar_scan is None:
            self._state.front_distance_cm = 0.0
            return

        self._state.front_distance_cm = rc_utils.get_lidar_average_distance(
            self._state.lidar_scan,
            0.0,
        )
