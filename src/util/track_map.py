""" Race track 2D grid """

from dataclasses import dataclass
import json
from pathlib import Path

import cv2 as cv
import numpy as np


@dataclass(frozen=True)
class TrackMap:
    grid: np.ndarray  # uint8, 0=free, 255=wall
    resolution_m_per_px: float
    world_min_x_m: float
    world_max_z_m: float
    width_px: int
    height_px: int

    @classmethod
    def load(cls, data_dir="data"):
        base = Path(data_dir)
        grid = np.load(base / "track_map.npy")
        with (base / "track_map.json").open() as f:
            meta = json.load(f)
        return cls(
            grid=grid,
            resolution_m_per_px=meta["resolution_m_per_px"],
            world_min_x_m=meta["world_min_x_m"],
            world_max_z_m=meta["world_max_z_m"],
            width_px=meta["width_px"],
            height_px=meta["height_px"],
        )

    # world (x_m, z_m) -> grid (row, col)
    def world_to_grid(self, x_m, z_m):
        col = int(round((x_m - self.world_min_x_m) / self.resolution_m_per_px))
        row = int(round((self.world_max_z_m - z_m) / self.resolution_m_per_px))
        return row, col

    # grid (row, col) -> world (x_m, z_m)
    def grid_to_world(self, row, col):
        x_m = self.world_min_x_m + col * self.resolution_m_per_px
        z_m = self.world_max_z_m - row * self.resolution_m_per_px
        return x_m, z_m

    def in_bounds(self, row, col):
        return 0 <= row < self.height_px and 0 <= col < self.width_px

    # Inflate walls by inflate_radius_m via distance transform O(n)
    def build_costmap(self, inflate_radius_m):
        radius_px = max(1.0, inflate_radius_m / self.resolution_m_per_px)
        free_mask = (self.grid == 0).astype(np.uint8)
        dist = cv.distanceTransform(free_mask, cv.DIST_L2, 5)
        return np.where(dist < radius_px, np.uint8(255), np.uint8(0))
