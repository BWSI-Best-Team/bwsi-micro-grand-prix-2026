from __future__ import annotations

import math
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
from localization.localizer import Localizer
from control.path_tracker import PurePursuitTracker
from control.stopper import Stopper
from util.track_map import TrackMap
from util.types import DriveCommand
from planning.global_planner.global_path import compute_multi_segment_path, GlobalPathConfig
from behavior.race_tree import build_race_tree, RaceContext


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

        # ICP localizer
        import json
        with open(DATA_DIR / "track_map.json") as f:
            meta = json.load(f)
        with open(DATA_DIR / "map_features.json") as f:
            feats = json.load(f)

        # ICP uses clean map (not nav map)
        _icp_map_path = DATA_DIR / "track_map_inference2.npy"
        if _icp_map_path.exists():
            grid = np.load(str(_icp_map_path)) > 0
        else:
            grid = self._path_track_map.grid > 0
        grid = np.ascontiguousarray(grid[::-1])  # flip row order
        origin = (meta["world_min_x_m"], meta["world_min_z_m"])
        res = meta["resolution_m_per_px"]

        # bake doors into grid as walls
        ox, oz = origin
        def _add_rect(g, d):
            hw, hh = d["size_x_m"] / 2, d["size_z_m"] / 2
            corners = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
            yaw = d.get("yaw_rad", 0)
            c_, s_ = math.cos(yaw), math.sin(yaw)
            R_ = np.array([[c_, -s_], [s_, c_]])
            cw = (R_ @ corners.T).T + np.array([d["center_x_m"], d["center_z_m"]])
            cols = ((cw[:, 0] - ox) / res).astype(int)
            rows = ((cw[:, 1] - oz) / res).astype(int)
            pts = np.column_stack([cols, rows]).reshape((-1, 1, 2))
            import cv2
            cv2.fillPoly(g.view(np.uint8), [pts], 1)
            return g.astype(bool)

        for key in ["OrangeWalls", "PurpWalls", "GreenWalls"]:
            for d in feats.get(key, []):
                if "size_x_m" in d:
                    grid = _add_rect(grid, d)

        # revolving door mask zones
        yellow_centers = [d for d in feats.get("YellowWalls", []) if d.get("name") == "center"]
        yellow_panels = [d for d in feats.get("YellowWalls", []) if "size_x_m" in d]
        mask_zones = []
        for yc in yellow_centers:
            cx, cz = yc["center_x_m"], yc["center_z_m"]
            max_r = 0
            for p in yellow_panels:
                dist = math.hypot(p["center_x_m"] - cx, p["center_z_m"] - cz)
                max_r = max(max_r, dist + p["size_x_m"] / 2)
            mask_zones.append((cx, cz, max(max_r, 1.0)))

        self._icp_loc = Localizer(grid, resolution=res, origin=origin,
                                   mask_zones=mask_zones)
        self._beam_angles = -np.linspace(0, 2 * np.pi, 720, endpoint=False)

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

        # open log file
        import csv
        log_dir = Path(__file__).resolve().parents[1] / "tmp"
        log_dir.mkdir(exist_ok=True)
        self._log_file = open(log_dir / "run_log.csv", "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(["frame", "phase", "speed", "angle", "x", "y", "yaw", "v_fwd", "front_cm"])

        self._bt_ctx.gate_enter_xy = GATE_ENTER_XY
        self._planning_done = False
        self._start_detected = False

    def _detect_start_position(self):
        color_img = self._rc.camera.get_color_image_no_copy()
        depth_img = self._rc.camera.get_depth_image()
        if color_img is None or depth_img is None:
            print(f"[START] no camera -> default A {START_XY_A}")
            return START_XY_A

        import cv2
        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([10, 150, 150]), np.array([30, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print(f"[START] no orange detected -> default A {START_XY_A}")
            return START_XY_A

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 200:
            print(f"[START] orange too small ({area:.0f}px) -> default A {START_XY_A}")
            return START_XY_A

        M = cv2.moments(largest)
        cy = int(M["m01"] / M["m00"])
        cx = int(M["m10"] / M["m00"])
        d_cm = depth_img[cy, cx] if 0 <= cy < 480 and 0 <= cx < 640 else 0

        if d_cm < 220:
            result = START_XY_B
        else:
            result = START_XY_A
        print(f"[START] orange at ({cx},{cy}) depth={d_cm:.0f}cm area={area:.0f}px -> {'A' if result == START_XY_A else 'B'} {result}")
        return result

    def _do_planning(self):
        if self._planning_done:
            return
        self._planning_done = True
        start_xy = self._detect_start_position()
        points = [start_xy, GATE_ENTER_XY, GATE_EXIT_XY, GOAL_XY]
        gate_cfg = GlobalPathConfig(inflate_m=GATE_INFLATE_M, corner_extra_m=CORNER_EXTRA_M)
        phase3_cfg = GlobalPathConfig(inflate_m=PHASE3_INFLATE_M, corner_extra_m=CORNER_EXTRA_M)
        cfgs = [None, gate_cfg, phase3_cfg]
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

    def update(self) -> None:
        # speed hack for phase 2+3
        if ENABLE_SIM_SPEED_HACK and self._bt_ctx.phase >= 2:
            import struct
            if hasattr(self._rc, "_RacecarSim__send_data"):
                self._rc._RacecarSim__send_data(
                    struct.pack("Bf",
                                self._rc.Header.drive_set_max_speed.value,
                                SIM_HACK_MAX_SPEED))
            else:
                self._rc.drive.set_max_speed(SIM_HACK_MAX_SPEED)
        else:
            self._rc.drive.set_max_speed(1.0)
        self._do_planning()

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

        if self._icp_loc.pose is not None:
            p = self._icp_loc.pose
            loc = f"icp=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})"
        else:
            loc = "icp=(?)"
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
        if getattr(self, "_path_preview_img", None) is None:
            return
        canvas = self._path_preview_img.copy()
        # add car position
        try:
            icp_p = self._icp_loc.pose
            if icp_p is None:
                raise ValueError
            x_m = icp_p[0]
            y_m = icp_p[1]
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

    def _get_pose(self):
        lidar = self._inputs.state.lidar_scan
        dt = max(self._rc.get_delta_time(), 0.001)
        gyro_z = self._inputs.state.imu_yaw_rate_rad_per_s

        # seed ICP on first frame
        if self._icp_loc.pose is None:
            start = self._detect_start_position()
            self._icp_loc.initialize_at(start[0], start[1], 0.0)
            print(f"[ICP] init at {start}")

        # ICP
        if lidar is not None:
            ranges_m = np.asarray(lidar, dtype=np.float32) / 100.0  # cm -> m
            try:
                accel_fwd = self._inputs.state.imu_accel_forward_mps2
                accel_right = self._inputs.state.imu_accel_right_mps2
                pose = self._icp_loc.update(
                    ranges_m, self._beam_angles, dt, gyro_z=gyro_z,
                    accel_forward=accel_fwd, accel_right=accel_right,
                    cmd_speed=self._speed, cmd_angle=self._angle)
                return pose[0], pose[1], pose[2]
            except Exception as e:
                if not getattr(self, '_icp_err_printed', False):
                    print(f"[WARNING] ICP failed: {e}")
                    self._icp_err_printed = True

        # fallback EKF
        if self._icp_loc.pose is not None:
            return self._icp_loc.pose[0], self._icp_loc.pose[1], self._icp_loc.pose[2]

        # fallback IMU
        p = self._pose_estimator.pose
        return p.x_m, p.y_m, p.yaw_rad

    def _compute_command(self) -> DriveCommand:
        if self._tracker is None:
            return DriveCommand(speed=0.0, angle=0.0)
        x_m, y_m, yaw = self._get_pose()

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
            if ctx._reset_detected:
                # reset always goes to position A
                self._planning_done = False
                self._icp_loc.initialize_at(START_XY_A[0], START_XY_A[1], 0.0)
        else:
            ctx._reset_detected = False
        ctx._last_x, ctx._last_y = x_m, y_m
        # door angle
        ctx.door_angle_rad = self._door_tracker.estimate_angle(
            ctx.lidar, x_m, y_m, yaw
        )
        # green detection
        self._color_detector.update(
            self._inputs.state.color_image, colors=[("GREEN", GREEN)]
        )
        self._depth_detector.update(self._inputs.state.depth_image)
        ctx.green_detected = self._color_detector.detected_color == "GREEN"
        # depth at green detection position
        if ctx.green_detected and self._color_detector.contour_center is not None:
            row, col = self._color_detector.contour_center
            depth_img = self._inputs.state.depth_image
            if depth_img is not None and 0 <= row < depth_img.shape[0] and 0 <= col < depth_img.shape[1]:
                d = depth_img[row, col]
                ctx.depth_center_cm = d if d > 0 else 9999.0
            else:
                ctx.depth_center_cm = 9999.0
        else:
            ctx.depth_center_cm = self._depth_detector.center_distance_cm
        ctx.speed, ctx.angle = 0.0, 0.0
        self._bt.tick(ctx)

        return DriveCommand(speed=ctx.speed, angle=ctx.angle)

    def _show_visualizer(self) -> None:
        import os, sys
        if sys.platform != "darwin" and os.environ.get("DISPLAY") is None:
            return  # headless (no X11), skip cv2 GUI

        if self._path_preview_img is not None:
            self._show_path_window()

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
