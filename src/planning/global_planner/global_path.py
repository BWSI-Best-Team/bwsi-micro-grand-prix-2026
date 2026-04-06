"""
Global path generation manager
    Dijkstra on costmap + Savitzky Golay smoothing
"""
from dataclasses import dataclass

import numpy as np
from scipy.signal import savgol_filter

from util.types import Waypoint
from util.constants import INFLATE_M, CORNER_EXTRA_M, SAVGOL_WINDOW_M, SMOOTH_PASSES
from .dijkstra_grid import GridDijkstraPlanner


@dataclass
class GlobalPathConfig:
    inflate_m: float = INFLATE_M
    plan_res_m: float = 0.15      # Dijkstra planning cell size
    resample_ds_m: float = 0.10   # output waypoint spacing
    savgol_window_m: float = SAVGOL_WINDOW_M
    savgol_polyorder: int = 3
    smooth_passes: int = SMOOTH_PASSES
    corner_extra_m: float = CORNER_EXTRA_M


def compute_global_path(track_map, start_xy, goal_xy, cfg=None):
    if cfg is None:
        cfg = GlobalPathConfig()

    xs_raw, ys_raw = _dijkstra_path(track_map, cfg, start_xy, goal_xy)
    xs_u, ys_u = _resample_arc(xs_raw, ys_raw, cfg.resample_ds_m)
    xs_s, ys_s = _smooth_xy(xs_u, ys_u, cfg)
    yaws = _compute_yaws(xs_s, ys_s, cfg.resample_ds_m)

    return [
        Waypoint(float(x), float(y), float(yaw))
        for x, y, yaw in zip(xs_s, ys_s, yaws)
    ]

# make a smooth path through several waypoints
def compute_multi_segment_path(track_map, waypoints_xy, cfgs=None):
    """Plan through multiple waypoints, smooth once at the end.
    waypoints_xy: list of (x, y) tuples, at least 2
    cfgs: list of GlobalPathConfig per segment (len = len(waypoints_xy)-1), or None for default
    """
    default_cfg = GlobalPathConfig()
    if cfgs is None:
        cfgs = [default_cfg] * (len(waypoints_xy) - 1)

    # collect raw dijkstra paths
    all_xs, all_ys = [], []
    for i in range(len(waypoints_xy) - 1):
        cfg = cfgs[i] or default_cfg
        xs, ys = _dijkstra_path(track_map, cfg, waypoints_xy[i], waypoints_xy[i + 1])
        if all_xs:
            xs, ys = xs[1:], ys[1:] # remove the duplicate waypoint
        all_xs.append(xs)
        all_ys.append(ys)

    xs_raw = np.concatenate(all_xs)
    ys_raw = np.concatenate(all_ys)

    # resample + smooth + yaws
    xs_u, ys_u = _resample_arc(xs_raw, ys_raw, default_cfg.resample_ds_m)
    xs_s, ys_s = _smooth_xy(xs_u, ys_u, default_cfg)
    yaws = _compute_yaws(xs_s, ys_s, default_cfg.resample_ds_m)

    return [
        Waypoint(float(x), float(y), float(yaw))
        for x, y, yaw in zip(xs_s, ys_s, yaws)
    ]





def _dijkstra_path(track_map, cfg, start_xy, goal_xy):
    # Build costmap inflation and downsample to grid
    costmap = track_map.build_costmap(cfg.inflate_m, cfg.corner_extra_m)
    scale = int(round(cfg.plan_res_m / track_map.resolution_m_per_px))
    h2, w2 = (costmap.shape[0] // scale) * scale, (costmap.shape[1] // scale) * scale
    coarse = costmap[:h2, :w2].reshape(h2 // scale, scale, w2 // scale, scale).max(axis=(1, 3))

    # Convert numpy grid to [x][y] bool list
    H, W = coarse.shape
    obs_xy = [[bool(coarse[H - 1 - y, x]) for y in range(H)] for x in range(W)]
    origin_y = track_map.world_max_z_m - (H - 1) * cfg.plan_res_m

    # Make the start and goal areas having no inflated zone around
    for px, py in [start_xy, goal_xy]:
        cx = round((px - track_map.world_min_x_m) / cfg.plan_res_m)
        cy = round((py - origin_y) / cfg.plan_res_m)
        for dx in range(-4, 5):
            for dy in range(-4, 5):
                gx, gy = cx + dx, cy + dy
                if 0 <= gx < W and 0 <= gy < H:
                    obs_xy[gx][gy] = False

    # Planning
    dj = GridDijkstraPlanner(obs_xy, track_map.world_min_x_m, origin_y, cfg.plan_res_m)
    rx, ry = dj.planning(start_xy[0], start_xy[1], goal_xy[0], goal_xy[1])
    return np.asarray(rx[::-1]), np.asarray(ry[::-1])


def _resample_arc(xs, ys, ds):
    seg = np.hypot(np.diff(xs), np.diff(ys))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    s_new = np.arange(0.0, s[-1], ds)
    xi = np.interp(s_new, s, xs)
    yi = np.interp(s_new, s, ys)
    return xi, yi


def _smooth_xy(xs, ys, cfg):
    # A legal savgol window that odd, <= n_samples, > polyorder
    win = int(round(cfg.savgol_window_m / cfg.resample_ds_m))
    if win % 2 == 0:
        win += 1
    win = min(win, len(xs) - (1 - len(xs) % 2))
    if win < cfg.savgol_polyorder + 2:
        win = cfg.savgol_polyorder + 3
        if win % 2 == 0:
            win += 1

    # Apply savgol
    for _ in range(cfg.smooth_passes):
        xs = savgol_filter(xs, window_length=win, polyorder=cfg.savgol_polyorder)
        ys = savgol_filter(ys, window_length=win, polyorder=cfg.savgol_polyorder)
    return xs, ys


def _compute_yaws(xs, ys, ds):
    dx = np.gradient(xs, ds)
    dy = np.gradient(ys, ds)
    return np.arctan2(dy, dx)
