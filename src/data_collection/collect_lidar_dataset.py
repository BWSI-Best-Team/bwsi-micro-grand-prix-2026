from __future__ import annotations

import csv
from datetime import datetime
import math
from pathlib import Path
import random
import sys
import time

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
STUDENT_ROOT = PROJECT_ROOT.parent
LIBRARY_DIR = STUDENT_ROOT / "library"
VENDOR_DIR = PROJECT_ROOT / "vendor"

sys.path.insert(0, str(VENDOR_DIR))
sys.path.insert(0, str(LIBRARY_DIR))

import racecar_core
from wall_map import build_grand_prix_wall_map, sample_valid_point


# Edit the seed before rerunning
SAMPLE_COUNT = 100
SETTLE_FRAMES = 4
RANDOM_SEED = 13371337
CELL_SIZE = 2.0
CLEARANCE = 3.0
OUTPUT_DIR = None
TELEPORT_SETTLE_DELAY_S = 0.5


rc = None
rng = random.Random(RANDOM_SEED)
frame_count = 0
sample_index = 0
wait_frames = 0
wait_until_s = 0.0
finished = False
spawn_y = 0.0
next_pose = None
wall_map = None
csv_file = None
csv_writer = None
output_dir = None
scans_dir = None


def get_output_dir() -> Path:
    global output_dir
    if OUTPUT_DIR is not None:
        output_dir = Path(OUTPUT_DIR)
        return output_dir

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = PROJECT_ROOT / "data" / f"lidar_samples_{timestamp}"
    return output_dir


def prepare_output_dir() -> None:
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {output_dir}")
    scans_dir.mkdir(parents=True, exist_ok=True)


def open_metadata_writer() -> None:
    global csv_file, csv_writer

    csv_file = (output_dir / "samples.csv").open("w", newline="", encoding="utf-8")
    csv_writer = csv.DictWriter(
        csv_file,
        fieldnames=[
            "sample_id",
            "requested_x",
            "requested_y",
            "requested_z",
            "requested_yaw_rad",
            "actual_x",
            "actual_y",
            "actual_z",
            "actual_yaw_rad",
            "scan_file",
            "scan_count",
            "scan_min",
            "scan_mean",
            "scan_max",
        ],
    )
    csv_writer.writeheader()


def schedule_next_capture() -> None:
    global next_pose, wait_frames, wait_until_s

    if sample_index >= SAMPLE_COUNT:
        finish_collection()
        return

    x, z = sample_valid_point(wall_map, rng)
    yaw_rad = rng.uniform(-math.pi, math.pi)
    next_pose = (x, spawn_y, z, yaw_rad)

    rc.physics.set_simulator_not_official_pose(x, spawn_y, z, yaw_rad)
    wait_frames = SETTLE_FRAMES
    wait_until_s = time.monotonic() + TELEPORT_SETTLE_DELAY_S


def save_sample(actual_pose: np.ndarray, scan_array: np.ndarray) -> None:
    sample_id = f"{sample_index:06d}"
    scan_filename = f"{sample_id}.npy"
    scan_path = scans_dir / scan_filename
    np.save(scan_path, scan_array)

    csv_writer.writerow(
        {
            "sample_id": sample_id,
            "requested_x": f"{next_pose[0]:.6f}",
            "requested_y": f"{next_pose[1]:.6f}",
            "requested_z": f"{next_pose[2]:.6f}",
            "requested_yaw_rad": f"{next_pose[3]:.6f}",
            "actual_x": f"{float(actual_pose[0]):.6f}",
            "actual_y": f"{float(actual_pose[1]):.6f}",
            "actual_z": f"{float(actual_pose[2]):.6f}",
            "actual_yaw_rad": f"{float(actual_pose[3]):.6f}",
            "scan_file": f"scans/{scan_filename}",
            "scan_count": int(scan_array.size),
            "scan_min": f"{float(scan_array.min()):.6f}",
            "scan_mean": f"{float(scan_array.mean()):.6f}",
            "scan_max": f"{float(scan_array.max()):.6f}",
        }
    )
    csv_file.flush()


def finish_collection() -> None:
    global finished, csv_file

    if finished:
        return

    finished = True
    rc.drive.stop()
    rc.drive.set_max_speed(0.0)

    if csv_file is not None:
        csv_file.close()
        csv_file = None

    print(f">> Collection complete. Captured {sample_index} samples in {output_dir}")


def start() -> None:
    global spawn_y, wall_map

    prepare_output_dir()

    try:
        seed_pose = rc.physics.get_simulator_not_official_pose()
    except NotImplementedError as error:
        raise RuntimeError(
            "This collector requires the unofficial simulator pose API."
        ) from error

    spawn_y = float(seed_pose[1])
    wall_map = build_grand_prix_wall_map(
        float(seed_pose[0]),
        float(seed_pose[2]),
        cell_size=CELL_SIZE,
        clearance=CLEARANCE,
    )

    open_metadata_writer()

    rc.drive.stop()
    rc.drive.set_max_speed(0.0)
    schedule_next_capture()

    print(">> LiDAR dataset collector")
    print(f"   output={output_dir}")
    print(f"   samples={SAMPLE_COUNT}")
    print(f"   reachable_cells={len(wall_map['reachable_cells'])}")


def update() -> None:
    global frame_count, sample_index, wait_frames

    frame_count += 1
    rc.drive.stop()
    rc.drive.set_max_speed(0.0)

    if finished:
        return

    if wait_frames > 0:
        wait_frames -= 1
        return

    # Only if the LiDAR can't catch up
    if time.monotonic() < wait_until_s:
        return

    if next_pose is None:
        schedule_next_capture()
        return

    lidar_scan = rc.lidar.get_samples()
    if lidar_scan is None:
        wait_frames = 1
        return

    actual_pose = rc.physics.get_simulator_not_official_pose()
    scan_array = np.asarray(lidar_scan, dtype=np.float32)
    scan_array = np.nan_to_num(scan_array, nan=0.0, posinf=0.0, neginf=0.0)

    save_sample(actual_pose, scan_array)
    sample_index += 1

    print(
        f"[collect] sample={sample_index}/{SAMPLE_COUNT} "
        f"x={float(actual_pose[0]):.3f} "
        f"z={float(actual_pose[2]):.3f} "
        f"yaw={float(actual_pose[3]):.3f}"
    )

    schedule_next_capture()


def update_slow() -> None:
    if finished:
        return

    remaining = SAMPLE_COUNT - sample_index
    print(
        f"[status] frame={frame_count} "
        f"captured={sample_index} "
        f"remaining={remaining}"
    )


def main() -> None:
    global rc, scans_dir

    get_output_dir()
    scans_dir = output_dir / "scans"

    rc = racecar_core.create_racecar()
    rc.set_start_update(start, update, update_slow)
    rc.go()


if __name__ == "__main__":
    main()
