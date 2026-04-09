import math
import numpy as np
from .map_manager import MapManager
from .icp import icp, scan_to_points, transform_points
from util.constants import ICP_CONFIG as CONFIG


class Localizer:

    def __init__(self, occupancy_grid, resolution=None, origin=(0.0, 0.0),
                 mask_zones=None):
        res = resolution or CONFIG["map_resolution"]
        self.map = MapManager(occupancy_grid, res, origin)
        self._mask_zones = np.array(mask_zones) if mask_zones else np.empty((0, 3))

        self.pose = None
        self.prev_pose = None
        self.prev_dt = 1.0 / 60
        self.sigma = CONFIG["icp_initial_sigma"]
        self._model_sse = 0.0
        self._model_n = 0
        self._lost_counter = 0

        # EKF state [x, z, theta, v, omega]
        self._ekf_x = np.zeros(5)
        self._ekf_P = np.eye(5) * 0.1
        self._Q = np.diag([0.001, 0.001, 0.001, 0.5, 0.1])
        self._R_base = np.diag([0.02, 0.02, 0.01])

        self.last_fitness = 0.0
        self.last_n_points = 0
        self.last_icp_correction = 0.0
        self.last_n_masked = 0

    def initialize_at(self, x, y, theta):
        self.pose = np.array([x, y, theta], dtype=np.float64)
        self.prev_pose = self.pose.copy()
        self.sigma = CONFIG["icp_initial_sigma"]
        self._model_sse = 0.0
        self._model_n = 0
        self._lost_counter = 0
        self._ekf_x = np.array([x, y, theta, 0.0, 0.0])
        self._ekf_P = np.eye(5) * 0.01

    def _in_bounds(self, pose):
        b = self.map.bounds
        m = 0.5
        return (b[0] - m < pose[0] < b[1] + m and
                b[2] - m < pose[1] < b[3] + m)

    def _ekf_predict(self, dt, gyro_z=None, accel_fwd=None, cmd_speed=None, cmd_angle=None):
        x, z, theta, v, omega = self._ekf_x

        # IMU acceleration drives speed
        if accel_fwd is not None:
            v = v + accel_fwd * dt

        # stopped detection
        if cmd_speed is not None and abs(cmd_speed) < 0.02:
            v *= 0.5

        v = np.clip(v, -10.0, 10.0)

        # gyro for heading
        if gyro_z is not None:
            omega = gyro_z

        # Ackermann kinematics
        x_new = x + v * np.cos(theta) * dt
        z_new = z + v * np.sin(theta) * dt
        theta_new = theta + omega * dt
        v_new = v
        self._ekf_x = np.array([x_new, z_new, theta_new, v_new, omega])

        # Jacobian
        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt

        self._ekf_P = F @ self._ekf_P @ F.T + self._Q * dt
        for i in range(5):
            self._ekf_P[i, i] = np.clip(self._ekf_P[i, i], 1e-6, 10.0)

    def _ekf_update(self, icp_pose, icp_H_matrix, fitness, n_masked, total_beams):
        z_meas = icp_pose[:3]
        H_obs = np.zeros((3, 5))
        H_obs[0, 0] = 1
        H_obs[1, 1] = 1
        H_obs[2, 2] = 1

        # adaptive R
        R = self._R_base.copy()

        if total_beams > 0:
            mask_ratio = n_masked / total_beams
            R *= (1.0 + mask_ratio * 10.0)

        R *= (1.0 + fitness * 5.0)

        n_points = total_beams - n_masked
        if n_points < 200:
            R *= max(1.0, 200.0 / max(n_points, 10))

        # degeneracy detection
        try:
            eigvals, eigvecs = np.linalg.eigh(icp_H_matrix)
            for i in range(3):
                if eigvals[i] < eigvals.max() * 0.01:
                    direction = eigvecs[:, i]
                    R += np.outer(direction, direction) * 10.0
        except:
            pass

        # Kalman gain
        S = H_obs @ self._ekf_P @ H_obs.T + R
        try:
            K = self._ekf_P @ H_obs.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        innovation = z_meas - H_obs @ self._ekf_x
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))

        # Mahalanobis gate
        try:
            maha_sq = innovation @ np.linalg.inv(S) @ innovation
            if maha_sq > 15.0:
                return
        except:
            pass

        self._ekf_x = self._ekf_x + K @ innovation
        self._ekf_P = (np.eye(5) - K @ H_obs) @ self._ekf_P
        self._ekf_x[2] = np.arctan2(np.sin(self._ekf_x[2]), np.cos(self._ekf_x[2]))
        self._ekf_x[3] = np.clip(self._ekf_x[3], -10.0, 10.0)

    def update(self, ranges, angles, dt=1/60, gyro_z=None,
               accel_forward=None, accel_right=None,
               cmd_speed=None, cmd_angle=None):
        if self.pose is None:
            return np.zeros(3)

        scan, _ = scan_to_points(ranges, angles)

        # revolving door DF-based filter: keep wall matches, mask panel hits
        n_masked = 0
        if len(scan) > 0 and len(self._mask_zones) > 0:
            world_pts = transform_points(scan, self.pose)
            d_check, _, _ = self.map.query(world_pts)
            in_zone = np.zeros(len(scan), dtype=bool)
            for cx, cz, r in self._mask_zones:
                in_zone |= ((world_pts[:, 0] - cx)**2 + (world_pts[:, 1] - cz)**2) < r * r
            keep = ~in_zone | (d_check < 0.15)
            n_masked = (~keep).sum()
            scan = scan[keep]

        self.last_n_points = len(scan)
        self.last_n_masked = n_masked
        total_beams = self.last_n_points + n_masked

        if len(scan) < 10:
            self.last_fitness = 10.0
            self._ekf_predict(dt, gyro_z, accel_forward, cmd_speed, cmd_angle)
            self.pose = self._ekf_x[:3].copy()
            return self.pose.copy()

        # subsample for speed
        if len(scan) > 300:
            scan = scan[::len(scan) // 300]

        # EKF predict
        self._ekf_predict(dt, gyro_z, accel_forward, cmd_speed, cmd_angle)
        predicted = self._ekf_x[:3].copy()

        # ICP
        result, fitness, H = icp(
            scan, predicted, self.map,
            sigma=self.sigma,
            max_iter=CONFIG["icp_max_iter"],
            converge_thresh=CONFIG["icp_converge_thresh"])

        self.last_fitness = fitness
        self.last_icp_correction = np.linalg.norm(result[:2] - predicted[:2])

        if not self._in_bounds(result) or self.last_icp_correction > 0.5:
            self.pose = predicted.copy()
            return self.pose.copy()

        # EKF update
        self._ekf_update(result, H, fitness, n_masked, total_beams)

        # adaptive sigma
        model_err = np.linalg.norm(result[:2] - predicted[:2])
        if model_err > 0.005:
            self._model_sse += model_err ** 2
            self._model_n += 1
            self.sigma = np.clip(np.sqrt(self._model_sse / self._model_n), 0.05, 0.3)

        # lost detection
        if fitness > CONFIG["lost_fitness_thresh"]:
            self._lost_counter += 1
            if self._lost_counter >= CONFIG["lost_frame_count"]:
                self._lost_counter = 0
                self.pose = predicted.copy()
                return self.pose.copy()
        else:
            self._lost_counter = 0

        # output EKF fused state
        self.prev_pose = self.pose.copy()
        self.prev_dt = dt
        self.pose = self._ekf_x[:3].copy()
        self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))
        return self.pose.copy()
