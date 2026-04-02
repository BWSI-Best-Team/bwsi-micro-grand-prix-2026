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
    imu_accel_right_mps2: float = 0.0
    imu_accel_forward_mps2: float = 0.0
    imu_yaw_rate_rad_per_s: float = 0.0


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
        self._update_imu_state()

        if self._state.lidar_scan is None:
            self._state.front_distance_cm = 0.0
            return

        self._state.front_distance_cm = rc_utils.get_lidar_average_distance(
            self._state.lidar_scan,
            0.0,
        )

    def _update_imu_state(self) -> None:
        accel = self._rc.physics.get_linear_acceleration()
        if accel is not None and len(accel) >= 3:
            self._state.imu_accel_right_mps2 = float(accel[0])
            self._state.imu_accel_forward_mps2 = float(accel[2])
        else:
            self._state.imu_accel_right_mps2 = 0.0
            self._state.imu_accel_forward_mps2 = 0.0

        gyro = self._rc.physics.get_angular_velocity()
        if gyro is not None and len(gyro) >= 2:
            # y axis is yaw in the simulator frame, positive = left
            self._state.imu_yaw_rate_rad_per_s = float(gyro[1])
        else:
            self._state.imu_yaw_rate_rad_per_s = 0.0
