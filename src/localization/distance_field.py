from __future__ import annotations

import math

import numpy as np


class DistanceField:
    def __init__(
        self,
        walls,
        cell_size_m: float = 0.05,
        padding_m: float = 2.0,
        max_distance_m: float = 5.0,
        wall_scale: float = 1.0,
    ) -> None:
        self.cell_size_m = cell_size_m
        self.max_distance_m = max_distance_m
        self.walls = []

        min_x_m = None
        max_x_m = None
        min_z_m = None
        max_z_m = None

        for wall in walls:
            x1_m, z1_m, x2_m, z2_m, half_thickness_m = parse_wall(wall, wall_scale)
            wall_data = {
                "x1_m": x1_m,
                "z1_m": z1_m,
                "x2_m": x2_m,
                "z2_m": z2_m,
                "half_thickness_m": half_thickness_m,
            }
            self.walls.append(wall_data)

            x_values = [
                x1_m - half_thickness_m,
                x2_m - half_thickness_m,
                x1_m + half_thickness_m,
                x2_m + half_thickness_m,
            ]
            z_values = [
                z1_m - half_thickness_m,
                z2_m - half_thickness_m,
                z1_m + half_thickness_m,
                z2_m + half_thickness_m,
            ]

            if min_x_m is None:
                min_x_m = min(x_values)
                max_x_m = max(x_values)
                min_z_m = min(z_values)
                max_z_m = max(z_values)
            else:
                min_x_m = min(min_x_m, min(x_values))
                max_x_m = max(max_x_m, max(x_values))
                min_z_m = min(min_z_m, min(z_values))
                max_z_m = max(max_z_m, max(z_values))

        self.min_x_m = min_x_m - padding_m
        self.max_z_m = max_z_m + padding_m
        max_x_m += padding_m
        min_z_m -= padding_m

        self.width_px = max(1, int(math.ceil((max_x_m - self.min_x_m) / self.cell_size_m)))
        self.height_px = max(1, int(math.ceil((self.max_z_m - min_z_m) / self.cell_size_m)))

        self.distance_m = np.zeros((self.height_px, self.width_px), dtype=np.float32)

        for row in range(self.height_px):
            for col in range(self.width_px):
                x_m, z_m = self.grid_to_world(row, col)
                best_distance_m = self.max_distance_m

                for wall in self.walls:
                    distance_to_center_m = point_to_segment_distance(
                        x_m,
                        z_m,
                        wall["x1_m"],
                        wall["z1_m"],
                        wall["x2_m"],
                        wall["z2_m"],
                    )
                    surface_distance_m = distance_to_center_m - wall["half_thickness_m"]
                    if surface_distance_m < 0.0:
                        surface_distance_m = 0.0
                    if surface_distance_m < best_distance_m:
                        best_distance_m = surface_distance_m

                self.distance_m[row, col] = best_distance_m

    def world_to_grid(self, x_m: float, z_m: float) -> tuple[int, int]:
        col = int(round((x_m - self.min_x_m) / self.cell_size_m))
        row = int(round((self.max_z_m - z_m) / self.cell_size_m))

        if col < 0:
            col = 0
        elif col >= self.width_px:
            col = self.width_px - 1

        if row < 0:
            row = 0
        elif row >= self.height_px:
            row = self.height_px - 1

        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        x_m = self.min_x_m + col * self.cell_size_m
        z_m = self.max_z_m - row * self.cell_size_m
        return x_m, z_m

    def in_bounds(self, x_m: float, z_m: float) -> bool:
        col = (x_m - self.min_x_m) / self.cell_size_m
        row = (self.max_z_m - z_m) / self.cell_size_m
        return 0.0 <= col < self.width_px and 0.0 <= row < self.height_px

    def distance_at(self, x_m: float, z_m: float) -> float:
        if not self.in_bounds(x_m, z_m):
            return float(self.max_distance_m)

        row, col = self.world_to_grid(x_m, z_m)
        return float(self.distance_m[row, col])

    def distance_at_bilinear_array(
        self,
        x_m_array: np.ndarray,
        z_m_array: np.ndarray,
    ) -> np.ndarray:
        x_arr = np.asarray(x_m_array, dtype=np.float32)
        z_arr = np.asarray(z_m_array, dtype=np.float32)
        if x_arr.shape != z_arr.shape:
            raise ValueError("x_m_array and z_m_array must have the same shape")
        if x_arr.size == 0:
            return np.zeros_like(x_arr, dtype=np.float32)

        col_f = (x_arr - self.min_x_m) / self.cell_size_m
        row_f = (self.max_z_m - z_arr) / self.cell_size_m

        in_bounds = (
            (col_f >= 0.0)
            & (col_f <= self.width_px - 1)
            & (row_f >= 0.0)
            & (row_f <= self.height_px - 1)
        )

        distances = np.full(x_arr.shape, self.max_distance_m, dtype=np.float32)
        if not np.any(in_bounds):
            return distances

        col0 = np.floor(col_f[in_bounds]).astype(np.int32)
        row0 = np.floor(row_f[in_bounds]).astype(np.int32)
        col1 = np.clip(col0 + 1, 0, self.width_px - 1)
        row1 = np.clip(row0 + 1, 0, self.height_px - 1)

        tc = col_f[in_bounds] - col0
        tr = row_f[in_bounds] - row0

        d00 = self.distance_m[row0, col0]
        d01 = self.distance_m[row0, col1]
        d10 = self.distance_m[row1, col0]
        d11 = self.distance_m[row1, col1]

        distances[in_bounds] = (
            d00 * (1.0 - tr) * (1.0 - tc)
            + d01 * (1.0 - tr) * tc
            + d10 * tr * (1.0 - tc)
            + d11 * tr * tc
        ).astype(np.float32)
        return distances


def parse_wall(wall, wall_scale: float) -> tuple[float, float, float, float, float]:
    if isinstance(wall, dict):
        x1_m = float(wall["x1_m"]) * wall_scale
        z1_m = float(wall["z1_m"]) * wall_scale
        x2_m = float(wall["x2_m"]) * wall_scale
        z2_m = float(wall["z2_m"]) * wall_scale
        half_thickness_m = float(wall["half_thickness_m"]) * wall_scale
    else:
        x1_m = float(wall.x1_m) * wall_scale
        z1_m = float(wall.z1_m) * wall_scale
        x2_m = float(wall.x2_m) * wall_scale
        z2_m = float(wall.z2_m) * wall_scale
        half_thickness_m = float(wall.half_thickness_m) * wall_scale

    return x1_m, z1_m, x2_m, z2_m, half_thickness_m


def point_to_segment_distance(
    x_m: float,
    z_m: float,
    x1_m: float,
    z1_m: float,
    x2_m: float,
    z2_m: float,
) -> float:
    segment_x_m = x2_m - x1_m
    segment_z_m = z2_m - z1_m
    segment_length_sq = segment_x_m * segment_x_m + segment_z_m * segment_z_m

    if segment_length_sq <= 1e-9:
        return math.hypot(x_m - x1_m, z_m - z1_m)

    dot = (x_m - x1_m) * segment_x_m + (z_m - z1_m) * segment_z_m
    progress = dot / segment_length_sq

    if progress < 0.0:
        progress = 0.0
    elif progress > 1.0:
        progress = 1.0

    closest_x_m = x1_m + progress * segment_x_m
    closest_z_m = z1_m + progress * segment_z_m
    return math.hypot(x_m - closest_x_m, z_m - closest_z_m)
