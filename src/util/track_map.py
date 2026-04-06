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
    # corner_extra_m is extra inflation for sharp corners
    def build_costmap(self, inflate_radius_m, corner_extra_m=0.0):
        radius_px = max(1.0, inflate_radius_m / self.resolution_m_per_px)
        free_mask = (self.grid == 0).astype(np.uint8)
        dist = cv.distanceTransform(free_mask, cv.DIST_L2, 5)
        costmap = np.where(dist < radius_px, np.uint8(255), np.uint8(0))

        if corner_extra_m > 0:
            # detect sharp corners on walls
            corners = cv.cornerHarris(self.grid, blockSize=5, ksize=3, k=0.04)
            corner_mask = (corners > 0.01 * corners.max()).astype(np.uint8) * 255
            extra_px = int(corner_extra_m / self.resolution_m_per_px)
            k = 2 * extra_px + 1
            corner_inflated = cv.dilate(corner_mask, np.ones((k, k), np.uint8))
            costmap = np.where(corner_inflated > 0, np.uint8(255), costmap)

        return costmap
