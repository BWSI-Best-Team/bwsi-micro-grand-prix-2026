from __future__ import annotations

import math
import struct
from enum import Enum, auto
from pathlib import Path
from typing import Any

import cv2 as cv
import numpy as np
from controller_config import ControllerConfig, load_controller_config
from perception.input_manager import InputManager
from perception.pose_estimator import PoseEstimator
from perception.door_tracker import DoorTracker
from perception.color_detector import ColorDetector, GREEN
from perception.depth_detector import DepthDetector
from control.path_tracker import PurePursuitTracker
from control.stopper import Stopper
from util.track_map import TrackMap
from util.types import DriveCommand
from planning.global_planner.global_path import compute_global_path, compute_multi_segment_path, GlobalPathConfig
from behavior.race_tree import build_race_tree, RaceContext
import racecar_utils as rc_utils


DATA_DIR = Path(__file__).resolve().parents[1] / "data"

from util.constants import *

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
        
        self._path_track_map = TrackMap.load(DATA_DIR)
        self._tracker = None # PurePursuitTracker
        self._bt = build_race_tree()
        self._bt_ctx = RaceContext()
        self._bt_ctx.stopper = Stopper()
        self._bt_ctx.rc = rc
        self._door_tracker = DoorTracker(DOOR_CENTER_XY, self._path_track_map)
        self._color_detector = ColorDetector()
        self._depth_detector = DepthDetector()

        self._path_preview_img = None
        self._log_file = None
        self._log_writer = None

    def start(self) -> None:
        self._frame_count = 0
        self._mode = RacecarState.RUNNING
        self._speed = 0.0
        self._angle = 0.0
        self._inputs.reset()
        self._pose_estimator.reset()
        self._rc.drive.stop()
        self._create_visualizer_windows()

        self._rc.set_update_slow_time(0.2) # print 5 times per second
        print(
            ">> BWSI 2026 Grand Prix Best Team controller\n"
        )
        if ENABLE_SIM_SPEED_HACK:
            print(f"[INFO] Simulator speed hack enabled: max_speed={SIM_HACK_MAX_SPEED:.2f}")

        # open log file
        import csv
        log_dir = Path(__file__).resolve().parents[1] / "tmp"
        log_dir.mkdir(exist_ok=True)
        self._log_file = open(log_dir / "run_log.csv", "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(["frame", "phase", "speed", "angle", "x", "y", "yaw", "v_fwd", "front_cm"])

        # set start position
#        self._rc.physics.set_simulator_not_official_pose(300.0, 0.0, 220.0, -np.pi / 2.0)

        # Global planing, now not using config but can using config file in the future, cause there are only two start points
        try:
            # Get position from API, needs to change to ML odom
            pose = self._rc.physics.get_simulator_not_official_pose()
            start_xy = (float(pose[0]) * UNITY_TO_M, float(pose[2]) * UNITY_TO_M)
        except AttributeError:
            print("[WARNING] Simulator pose API not available, using default start")
            start_xy = (17.68, 24.80)
        # plan through all waypoints and smooth only once
        points = [start_xy, GATE_ENTER_XY, GATE_EXIT_XY, GOAL_XY]
        gate_cfg = GlobalPathConfig(inflate_m=GATE_INFLATE_M, corner_extra_m=CORNER_EXTRA_M)
        cfgs = [None, gate_cfg, None]  # default for seg1 and seg3, tighter for seg2
        print(f"[INFO] Global Planner: {' -> '.join(str(p) for p in points)}")
        waypoints = compute_multi_segment_path(self._path_track_map, points, cfgs)
        self._tracker = PurePursuitTracker(
            waypoints,
            cruise_speed=CRUISE_SPEED,
            curve_slowdown=CURVE_SLOWDOWN,
            curve_horizon_m=CURVE_HORIZON_M,
        )
        print(f"[INFO] Global path: {len(waypoints)} waypoints")
        self._save_path_preview(self._path_track_map, waypoints, start_xy)

        # spin door location for behavior tree
        self._bt_ctx.gate_enter_xy = GATE_ENTER_XY

    def update(self) -> None:
        self._apply_drive_max_speed()

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

        # log every frame
        if self._log_writer:
            ctx = self._bt_ctx
            self._log_writer.writerow([
                self._frame_count, ctx.phase,
                f"{self._speed:.3f}", f"{self._angle:.3f}",
                f"{ctx.x:.3f}", f"{ctx.y:.3f}", f"{ctx.yaw:.3f}",
                f"{ctx.v_fwd:.3f}",
                f"{self._inputs.state.front_distance_cm:.1f}",
            ])
            self._log_file.flush()
        self._rc.drive.set_speed_angle(self._speed, self._angle)

    def update_slow(self) -> None: # For logging only
        if not self._config.print_log:
            return

        try:
            pose = self._rc.physics.get_simulator_not_official_pose()
            yaw_ours = np.pi / 2.0 - float(pose[3])
            loc = f"gt=({pose[0]*UNITY_TO_M:.2f},{pose[2]*UNITY_TO_M:.2f},{yaw_ours:.2f})"
        except AttributeError:
            p = self._pose_estimator.pose
            loc = f"imu=({p.x_m:.2f},{p.y_m:.2f},{p.yaw_rad:.2f})"
        phase_names = {1: "TO_GATE", 2: "PASS_GATE", 3: "TO_FINISH"}
        phase = phase_names.get(self._bt_ctx.phase, "?")
        print(
            f"[Phase {self._bt_ctx.phase}: {phase}] "
            f"cmd=({self._speed:.2f},{self._angle:.2f}) "
            f"front={self._inputs.state.front_distance_cm:.1f}cm "
            f"{loc} "
            f"{self._door_tracker.debug_str()}"
        )

    def _save_path_preview(self, track_map, path, start_xy) -> None:
        """Render the planned global path, save + open it in a window."""
        canvas = cv.cvtColor(track_map.grid, cv.COLOR_GRAY2BGR)
        for i in range(len(path) - 1):
            r0, c0 = track_map.world_to_grid(path[i].x_m, path[i].y_m)
            r1, c1 = track_map.world_to_grid(path[i + 1].x_m, path[i + 1].y_m)
            cv.line(canvas, (c0, r0), (c1, r1), (255, 255, 0), 3)
        # points: green=start, yellow=gate, red=finish
        sr, sc = track_map.world_to_grid(*start_xy)
        gatr, gatc = track_map.world_to_grid(*GATE_ENTER_XY)
        gr, gc = track_map.world_to_grid(*GOAL_XY)
        cv.circle(canvas, (sc, sr), 15, (0, 255, 0), -1)
        cv.circle(canvas, (gatc, gatr), 15, (0, 255, 255), -1)
        cv.circle(canvas, (gc, gr), 15, (0, 0, 255), -1)
        scale = 1400 / canvas.shape[1]
        preview = cv.resize(
            canvas,
            (int(canvas.shape[1] * scale), int(canvas.shape[0] * scale)),
            interpolation=cv.INTER_AREA,
        )
        tmp_dir = Path(__file__).resolve().parents[1] / "tmp"
        tmp_dir.mkdir(exist_ok=True)
        out_path = tmp_dir / "planned_path_preview.png"
        cv.imwrite(str(out_path), preview)
        self._path_preview_img = preview
        print(f"[INFO] Path preview: {out_path}")

    def _show_path_window(self) -> None:
        """Show the planned path (and live car pose) in an OpenCV window."""
        if getattr(self, "_path_preview_img", None) is None:
            return
        canvas = self._path_preview_img.copy()
        # add car position
        try:
            pose = self._rc.physics.get_simulator_not_official_pose()
            x_m = float(pose[0]) * UNITY_TO_M
            y_m = float(pose[2]) * UNITY_TO_M
            # world -> pixels
            tm = self._path_track_map
            orig_r, orig_c = tm.world_to_grid(x_m, y_m)
            scale = canvas.shape[1] / tm.width_px
            pr = int(orig_r * scale)
            pc = int(orig_c * scale)
            cv.circle(canvas, (pc, pr), 8, (0, 200, 255), -1)  # orange dot
        except Exception:
            pass
        # draw door blades
        self._draw_door_blades(canvas)
        cv.imshow("BWSI Path", canvas)

    def _draw_door_blades(self, canvas) -> None:
        tm = self._path_track_map
        scale = canvas.shape[1] / tm.width_px
        dt = self._door_tracker

        # draw detected blade points with green dots
        for wx, wy in dt._blade_points_world:
            r, c = tm.world_to_grid(wx, wy)
            px, py = int(c * scale), int(r * scale)
            cv.circle(canvas, (px, py), 3, (0, 255, 0), -1)

        # draw door center
        dr, dc = tm.world_to_grid(dt.cx, dt.cy)
        cx_px, cy_px = int(dc * scale), int(dr * scale)
        cv.circle(canvas, (cx_px, cy_px), 6, (0, 255, 255), -1)

        # draw estimated blade lines
        if dt._debug_angle is not None:
            blade_len_px = int(1.0 / tm.resolution_m_per_px * scale) # 1m line
            for k in range(4):
                a = dt._debug_angle + k * math.pi / 2.0
                ex = dt.cx + 1.0 * math.cos(a)
                ey = dt.cy + 1.0 * math.sin(a)
                er, ec = tm.world_to_grid(ex, ey)
                ep = (int(ec * scale), int(er * scale))
                cv.line(canvas, (cx_px, cy_px), ep, (0, 0, 255), 2)

    def _update_mode(self) -> None:
        if self._inputs.state.color_image is None:
            self._mode = RacecarState.INITIALIZING
        else:
            self._mode = RacecarState.RUNNING

    def _compute_command(self) -> DriveCommand:
        if self._tracker is None:
            return DriveCommand(speed=0.0, angle=0.0)
        # use sim patch if available, else IMU estimator
        try:
            pose = self._rc.physics.get_simulator_not_official_pose()
        except AttributeError:
            p = self._pose_estimator.pose
            x_m, y_m, yaw = p.x_m, p.y_m, p.yaw_rad
        else:
            # Unity yaw=0 north, left-handed
            # Ours yaw=0 east, right-handed
            x_m = float(pose[0]) * UNITY_TO_M
            y_m = float(pose[2]) * UNITY_TO_M
            yaw = np.pi / 2.0 - float(pose[3])

        # behavior tree decides speed + angle
        ctx = self._bt_ctx
        ctx.tracker = self._tracker
        ctx.x, ctx.y, ctx.yaw = x_m, y_m, yaw
        ctx.lidar = self._inputs.state.lidar_scan
        ctx.dt = max(self._rc.get_delta_time(), 0.001)
        self._pose_estimator.update_speed_from_position(x_m, y_m, ctx.dt)
        ctx.v_fwd = self._pose_estimator.speed_mps

        # update last position
        if hasattr(ctx, '_last_x'):
            jump = math.hypot(x_m - ctx._last_x, y_m - ctx._last_y)
            ctx._reset_detected = jump > 5.0
        else:
            ctx._reset_detected = False
        ctx._last_x, ctx._last_y = x_m, y_m
        # door angle estimation
        ctx.door_angle_rad = self._door_tracker.estimate_angle(
            ctx.lidar, x_m, y_m, yaw
        )
        # color + depth detection
        self._color_detector.update(
            self._inputs.state.color_image, colors=[("GREEN", GREEN)]
        )
        self._depth_detector.update(self._inputs.state.depth_image)
        ctx.green_detected = self._color_detector.detected_color == "GREEN"
        ctx.depth_center_cm = self._depth_detector.center_distance_cm
        ctx.speed, ctx.angle = 0.0, 0.0
        self._bt.tick(ctx)

        return DriveCommand(speed=ctx.speed, angle=ctx.angle)

    def _apply_drive_max_speed(self) -> None:
        if ENABLE_SIM_SPEED_HACK and hasattr(self._rc, "_RacecarSim__send_data"):
            self._rc._RacecarSim__send_data(
                struct.pack(
                    "Bf",
                    self._rc.Header.drive_set_max_speed.value,
                    SIM_HACK_MAX_SPEED,
                )
            )
            return

        self._rc.drive.set_max_speed(1.0)

    def _show_visualizer(self) -> None:
        # show path window
        if self._path_preview_img is not None:
            self._show_path_window()

        if not self._config.show_visualizer:
            cv.waitKey(1)
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
        # Reconstruct field view from 1D array
        height = self.LIDAR_WINDOW_SIZE[1]
        width = self.LIDAR_WINDOW_SIZE[0]
        image = np.zeros((height, width, 3), dtype=np.uint8)

        samples = np.nan_to_num(
            self._inputs.state.lidar_scan,
            nan=0.0,
            posinf=self.LIDAR_DISPLAY_MAX_CM,
            neginf=0.0,
        )
        samples = np.clip(samples, 0.0, self.LIDAR_DISPLAY_MAX_CM)

        center_x = width // 2
        center_y = height // 2
        max_radius_px = min(width, height) // 2 - 10
        angle_step_rad = 2.0 * np.pi / len(samples)

        for index, distance_cm in enumerate(samples):
            if distance_cm > 0.0: # Sanitize
                angle_rad = index * angle_step_rad
                radius_px = distance_cm / self.LIDAR_DISPLAY_MAX_CM * max_radius_px
                col = int(center_x + radius_px * np.sin(angle_rad))
                row = int(center_y - radius_px * np.cos(angle_rad))

                if 0 <= row < height and 0 <= col < width:
                    cv.circle(image, (col, row), 2, (0, 0, 255), -1)

        return image
