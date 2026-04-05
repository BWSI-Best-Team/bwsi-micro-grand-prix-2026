from __future__ import annotations

from collections import deque
import math
import random

from grand_prix_walls import GRAND_PRIX_WALLS, Wall


def build_grand_prix_wall_map(
    seed_x: float,
    seed_z: float,
    cell_size: float = 2.0,
    clearance: float = 3.0,
    padding: float = 4.0,
) -> dict:
    wall_map: dict = {}
    wall_map["walls"] = GRAND_PRIX_WALLS
    wall_map["cell_size"] = cell_size
    wall_map["clearance"] = clearance
    wall_map["padding"] = padding

    min_x = None
    max_x = None
    min_z = None
    max_z = None

    for wall in wall_map["walls"]:
        reach = wall.half_thickness_m + clearance

        x_values = [
            wall.x1_m - reach,
            wall.x2_m - reach,
            wall.x1_m + reach,
            wall.x2_m + reach,
        ]
        z_values = [
            wall.z1_m - reach,
            wall.z2_m - reach,
            wall.z1_m + reach,
            wall.z2_m + reach,
        ]

        if min_x is None:
            min_x = min(x_values)
            max_x = max(x_values)
            min_z = min(z_values)
            max_z = max(z_values)
        else:
            min_x = min(min_x, min(x_values))
            max_x = max(max_x, max(x_values))
            min_z = min(min_z, min(z_values))
            max_z = max(max_z, max(z_values))

    wall_map["min_x"] = min_x - padding
    wall_map["max_x"] = max_x + padding
    wall_map["min_z"] = min_z - padding
    wall_map["max_z"] = max_z + padding

    wall_map["width"] = max(
        1,
        math.ceil((wall_map["max_x"] - wall_map["min_x"]) / cell_size),
    )
    wall_map["height"] = max(
        1,
        math.ceil((wall_map["max_z"] - wall_map["min_z"]) / cell_size),
    )

    blocked = []
    reachable = []

    for _ in range(wall_map["height"]):
        blocked_row = []
        reachable_row = []
        for _ in range(wall_map["width"]):
            blocked_row.append(False)
            reachable_row.append(False)
        blocked.append(blocked_row)
        reachable.append(reachable_row)

    wall_map["blocked"] = blocked
    wall_map["reachable"] = reachable
    wall_map["reachable_cells"] = []

    for cell_z in range(wall_map["height"]):
        for cell_x in range(wall_map["width"]):
            x, z = cell_center(wall_map, cell_x, cell_z)
            wall_map["blocked"][cell_z][cell_x] = point_hits_wall(wall_map, x, z)

    start_cell = find_nearest_open_cell(wall_map, seed_x, seed_z)
    if start_cell is None:
        raise RuntimeError("Could not find a reachable seed cell for the wall map.")

    queue: deque[tuple[int, int]] = deque()
    queue.append(start_cell)

    start_x, start_z = start_cell
    wall_map["reachable"][start_z][start_x] = True
    wall_map["reachable_cells"].append(start_cell)

    while queue:
        cell_x, cell_z = queue.popleft()

        for next_x, next_z in neighbors(wall_map, cell_x, cell_z):
            if wall_map["blocked"][next_z][next_x]:
                continue

            if wall_map["reachable"][next_z][next_x]:
                continue

            wall_map["reachable"][next_z][next_x] = True
            wall_map["reachable_cells"].append((next_x, next_z))
            queue.append((next_x, next_z))

    return wall_map


def is_valid_point(wall_map: dict, x: float, z: float) -> bool:
    cell = point_to_cell(wall_map, x, z)
    if cell is None:
        return False

    cell_x, cell_z = cell
    if not wall_map["reachable"][cell_z][cell_x]:
        return False

    if point_hits_wall(wall_map, x, z):
        return False

    return True


def sample_valid_point(wall_map: dict, rng: random.Random) -> tuple[float, float]:
    if not wall_map["reachable_cells"]:
        raise RuntimeError("Wall map does not contain any reachable cells.")

    cell_x, cell_z = rng.choice(wall_map["reachable_cells"])
    return cell_center(wall_map, cell_x, cell_z)


def find_nearest_open_cell(
    wall_map: dict,
    seed_x: float,
    seed_z: float,
) -> tuple[int, int] | None:
    seed_cell = point_to_cell(wall_map, seed_x, seed_z)

    if seed_cell is None:
        cell_x = int((seed_x - wall_map["min_x"]) / wall_map["cell_size"])
        cell_z = int((seed_z - wall_map["min_z"]) / wall_map["cell_size"])

        if cell_x < 0:
            cell_x = 0
        if cell_x >= wall_map["width"]:
            cell_x = wall_map["width"] - 1
        if cell_z < 0:
            cell_z = 0
        if cell_z >= wall_map["height"]:
            cell_z = wall_map["height"] - 1

        seed_cell = (cell_x, cell_z)

    max_radius = max(wall_map["width"], wall_map["height"])

    for radius in range(max_radius):
        min_x = max(seed_cell[0] - radius, 0)
        max_x = min(seed_cell[0] + radius, wall_map["width"] - 1)
        min_z = max(seed_cell[1] - radius, 0)
        max_z = min(seed_cell[1] + radius, wall_map["height"] - 1)

        for cell_z in range(min_z, max_z + 1):
            for cell_x in range(min_x, max_x + 1):
                if not wall_map["blocked"][cell_z][cell_x]:
                    return cell_x, cell_z

    return None


def point_to_cell(wall_map: dict, x: float, z: float) -> tuple[int, int] | None:
    if x < wall_map["min_x"] or x >= wall_map["max_x"]:
        return None
    if z < wall_map["min_z"] or z >= wall_map["max_z"]:
        return None

    cell_x = int((x - wall_map["min_x"]) / wall_map["cell_size"])
    cell_z = int((z - wall_map["min_z"]) / wall_map["cell_size"])
    return cell_x, cell_z


def cell_center(wall_map: dict, cell_x: int, cell_z: int) -> tuple[float, float]:
    x = wall_map["min_x"] + (cell_x + 0.5) * wall_map["cell_size"]
    z = wall_map["min_z"] + (cell_z + 0.5) * wall_map["cell_size"]
    return x, z


def neighbors(wall_map: dict, cell_x: int, cell_z: int) -> list[tuple[int, int]]:
    out = []

    if cell_x + 1 < wall_map["width"]:
        out.append((cell_x + 1, cell_z))
    if cell_x - 1 >= 0:
        out.append((cell_x - 1, cell_z))
    if cell_z + 1 < wall_map["height"]:
        out.append((cell_x, cell_z + 1))
    if cell_z - 1 >= 0:
        out.append((cell_x, cell_z - 1))

    return out


def point_hits_wall(wall_map: dict, x: float, z: float) -> bool:
    for wall in wall_map["walls"]:
        wall_radius = wall.half_thickness_m + wall_map["clearance"]
        distance = point_to_segment_distance(x, z, wall)
        if distance <= wall_radius:
            return True

    return False


def point_to_segment_distance(x: float, z: float, wall: Wall) -> float:
    segment_x = wall.x2_m - wall.x1_m
    segment_z = wall.z2_m - wall.z1_m
    segment_length_sq = segment_x * segment_x + segment_z * segment_z

    if segment_length_sq <= 0.0:
        return math.hypot(x - wall.x1_m, z - wall.z1_m)

    dot = (x - wall.x1_m) * segment_x + (z - wall.z1_m) * segment_z
    progress = dot / segment_length_sq

    if progress < 0.0:
        progress = 0.0
    if progress > 1.0:
        progress = 1.0

    closest_x = wall.x1_m + progress * segment_x
    closest_z = wall.z1_m + progress * segment_z
    return math.hypot(x - closest_x, z - closest_z)
