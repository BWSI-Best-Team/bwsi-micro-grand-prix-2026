import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates


def _build_df(grid, resolution):
    df = distance_transform_edt(~grid).astype(np.float32) * resolution
    gy, gx = np.gradient(df)
    norm = np.sqrt(gx ** 2 + gy ** 2).clip(1e-8)
    nx = (gx / norm).astype(np.float32)
    ny = (gy / norm).astype(np.float32)
    return df, nx, ny


class MapManager:

    def __init__(self, occupancy_grid, resolution=0.01, origin=(0.0, 0.0)):
        self.resolution = resolution
        self.origin = np.array(origin, dtype=np.float64)
        self.grid = occupancy_grid.astype(bool)
        self.df, self.nx, self.ny = _build_df(self.grid, resolution)

        h, w = self.grid.shape
        self.bounds = np.array([
            origin[0], origin[0] + w * resolution,
            origin[1], origin[1] + h * resolution
        ])

    def query(self, world_points):
        px = (world_points[:, 0] - self.origin[0]) / self.resolution
        py = (world_points[:, 1] - self.origin[1]) / self.resolution
        coords = np.array([py, px])

        d = map_coordinates(self.df, coords, order=1, mode='constant', cval=10.0)
        nx = map_coordinates(self.nx, coords, order=1, mode='constant', cval=0.0)
        ny = map_coordinates(self.ny, coords, order=1, mode='constant', cval=0.0)
        return d, nx, ny

    def is_free(self, world_points, threshold=0.05):
        d, _, _ = self.query(world_points)
        return d > threshold

    def sample_free_space(self, n_points):
        xmin, xmax, ymin, ymax = self.bounds
        points = []
        while len(points) < n_points:
            batch = np.column_stack([
                np.random.uniform(xmin, xmax, n_points),
                np.random.uniform(ymin, ymax, n_points),
            ])
            points.extend(batch[self.is_free(batch)].tolist())
        return np.array(points[:n_points])
