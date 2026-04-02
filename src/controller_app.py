from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any

import cv2 as cv
import numpy as np
from controller_config import ControllerConfig, load_controller_config
from input_manager import InputManager
from pose_estimator import PoseEstimator
import racecar_utils as rc_utils


@dataclass(slots=True)
class DriveCommand:
    speed: float = 0.0
    angle: float = 0.0


class RacecarState(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    RUNNING = auto()


class GrandPrixController:
    COLOR_WINDOW = "BWSI Color"
    DEPTH_WINDOW = "BWSI Depth"
    LIDAR_WINDOW = "BWSI Lidar"
    COLOR_WINDOW_SIZE = (640, 480)
    DEPTH_WINDOW_SIZE = (640, 480)
    LIDAR_WINDOW_SIZE = (640, 240)
    LIDAR_DISPLAY_MAX_CM = 1000.0

    def __init__(self, rc: Any) -> None:
        self._rc = rc
        self._config: ControllerConfig = load_controller_config()
        self._frame_count = 0
        self._mode = RacecarState.IDLE
        self._speed = 0.0
        self._angle = 0.0
        self._inputs = InputManager(rc)
        self._pose_estimator = PoseEstimator()

    def start(self) -> None:
        self._frame_count = 0
        self._mode = RacecarState.RUNNING
        self._speed = 0.0
        self._angle = 0.0
        self._inputs.reset()
        self._pose_estimator.reset()
        self._rc.drive.stop()
        self._create_visualizer_windows()

        print(
            ">> BWSI 2026 Grand Prix Best Team controller\n"
        )

    def update(self) -> None:
        self._rc.drive.set_max_speed(1.0) # Located here per regulation

        self._frame_count += 1
        self._inputs.update()
        self._pose_estimator.update(
            self._inputs.state.imu_accel_right_mps2,
            self._inputs.state.imu_accel_forward_mps2,
            self._inputs.state.imu_yaw_rate_rad_per_s,
            self._rc.get_delta_time(),
        )
        self._show_visualizer()
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
            f"front={self._inputs.state.front_distance_cm:.1f}cm "
            f"color={'yes' if self._inputs.state.color_image is not None else 'no'} "
            f"location={self._pose_estimator.pose.x_m}, {self._pose_estimator.pose.y_m}"
        )

    def _update_mode(self) -> None:
        if self._inputs.state.color_image is None:
            self._mode = RacecarState.INITIALIZING
        else:
            self._mode = RacecarState.RUNNING

    def _compute_command(self) -> DriveCommand:
        # TODO: add real controller when perception and policy are ready.
        return DriveCommand(speed=1.0, angle=0.0)

    def _show_visualizer(self) -> None:
        if not self._config.show_visualizer:
            return

        if self._inputs.state.color_image is not None:
            cv.imshow(self.COLOR_WINDOW, self._inputs.state.color_image)

        if self._inputs.state.depth_image is not None:
            cv.imshow(
                self.DEPTH_WINDOW,
                rc_utils.colormap_depth_image(self._inputs.state.depth_image),
            )

        if self._inputs.state.lidar_scan is not None:
            cv.imshow(self.LIDAR_WINDOW, self._make_lidar_view())

        cv.waitKey(1)

    def _create_visualizer_windows(self) -> None:
        if not self._config.show_visualizer:
            return

        cv.namedWindow(self.COLOR_WINDOW, cv.WINDOW_NORMAL | cv.WINDOW_FREERATIO)
        cv.namedWindow(self.DEPTH_WINDOW, cv.WINDOW_NORMAL | cv.WINDOW_FREERATIO)
        cv.namedWindow(self.LIDAR_WINDOW, cv.WINDOW_NORMAL | cv.WINDOW_FREERATIO)
        cv.resizeWindow(self.COLOR_WINDOW, *self.COLOR_WINDOW_SIZE)
        cv.resizeWindow(self.DEPTH_WINDOW, *self.DEPTH_WINDOW_SIZE)
        cv.resizeWindow(self.LIDAR_WINDOW, *self.LIDAR_WINDOW_SIZE)

    def _make_lidar_view(self) -> np.ndarray:
        # LiDAR is a 1D distance scan, so scale it into a thicker strip for display.
        samples = np.nan_to_num(
            self._inputs.state.lidar_scan,
            nan=0.0,
            posinf=self.LIDAR_DISPLAY_MAX_CM,
            neginf=0.0,
        )
        samples = np.clip(samples, 0.0, self.LIDAR_DISPLAY_MAX_CM)
        row = (255.0 * samples / self.LIDAR_DISPLAY_MAX_CM).astype(np.uint8)
        return np.repeat(row[np.newaxis, :], self.LIDAR_WINDOW_SIZE[1], axis=0)
