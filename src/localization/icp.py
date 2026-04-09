# Based on CSM and KISS-ICP

import numpy as np


def transform_points(points_local, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return points_local @ R.T + pose[:2]


def scan_to_points(ranges, angles, max_range=25.0):
    valid = np.isfinite(ranges) & (ranges > 0.01) & (ranges < max_range)
    r_valid = ranges[valid]
    a_valid = angles[valid]
    if len(r_valid) == 0:
        return np.empty((0, 2)), valid
    points = np.column_stack([r_valid * np.cos(a_valid),
                              r_valid * np.sin(a_valid)])
    return points, valid


def _icp_core(scan_local, pose_guess, map_mgr, sigma=0.5,
              max_iter=15, converge_thresh=1e-4):
    pose = pose_guess.astype(np.float64).copy()
    N = len(scan_local)
    s2 = sigma ** 2
    H = np.eye(3)

    if N < 3:
        return pose, 10.0, H

    for _ in range(max_iter):
        world_pts = transform_points(scan_local, pose)
        d, nx, ny = map_mgr.query(world_pts)

        # GM kernel weights
        d2 = d ** 2
        w_gm = s2 * s2 / (s2 + d2) ** 2

        # incidence angle weights
        beam_dirs = world_pts - pose[:2]
        beam_len = np.linalg.norm(beam_dirs, axis=1).clip(1e-8)
        beam_dirs = beam_dirs / beam_len[:, None]
        cos_beta = np.abs(nx * beam_dirs[:, 0] + ny * beam_dirs[:, 1])
        w = w_gm * (cos_beta ** 2)

        # 3x3 normal equation
        c, s = np.cos(pose[2]), np.sin(pose[2])
        J_theta = (nx * (-s * scan_local[:, 0] - c * scan_local[:, 1]) +
                   ny * (c * scan_local[:, 0] - s * scan_local[:, 1]))
        J = np.column_stack([nx, ny, J_theta])
        Jw = J * w[:, None]
        H = Jw.T @ J
        b = Jw.T @ d

        try:
            delta = np.linalg.solve(H, -b)
        except np.linalg.LinAlgError:
            break

        pose += delta
        if np.linalg.norm(delta) < converge_thresh:
            break

    # fitness
    d_final, _, _ = map_mgr.query(transform_points(scan_local, pose))
    w_final = s2 * s2 / (s2 + d_final ** 2) ** 2
    w_sum = w_final.sum()
    fitness = np.average(np.abs(d_final), weights=w_final) if w_sum > 1e-10 else 10.0

    return pose, fitness, H


def icp(scan_local, pose_guess, map_mgr, sigma=0.5,
        max_iter=15, converge_thresh=1e-4):
    if len(scan_local) < 3:
        return pose_guess.astype(np.float64).copy(), 10.0, np.eye(3)

    return _icp_core(scan_local, pose_guess, map_mgr, sigma, max_iter, converge_thresh)
