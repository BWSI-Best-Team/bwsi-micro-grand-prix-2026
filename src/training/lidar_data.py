import csv
import math
import random
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = PROJECT_ROOT / "data"


class LidarTrainingDataset(Dataset):
    def __init__(
        self,
        dataset_dir: Path,
        rows: List[Dict[str, object]],
        scaler: Dict[str, float],
        max_range_cm: float,
        num_beams: int,
    ) -> None:
        self.dataset_dir = dataset_dir
        self.rows = rows
        self.scaler = scaler
        self.max_range_cm = max_range_cm
        self.num_beams = num_beams

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> Tuple[torch.Tensor, torch.Tensor]:
        row = self.rows[index]
        scan_path = self.dataset_dir / str(row["scan_file"])
        scan = np.load(scan_path).astype(np.float32)

        if scan.size != self.num_beams:
            raise ValueError(
                f"Unexpected LiDAR length for {scan_path}: "
                f"expected {self.num_beams}, got {scan.size}"
            )

        scan = self._clean_scan(scan)

        x_value = float(row["x"])
        z_value = float(row["z"])
        yaw_value = float(row["yaw"])

        target = np.array(
            [
                (x_value - self.scaler["x_mean"]) / self.scaler["x_std"],
                (z_value - self.scaler["z_mean"]) / self.scaler["z_std"],
                math.sin(yaw_value),
                math.cos(yaw_value),
            ],
            dtype=np.float32,
        )

        scan_tensor = torch.from_numpy(scan).unsqueeze(0)
        target_tensor = torch.from_numpy(target)
        return scan_tensor, target_tensor

    def _clean_scan(self, scan: np.ndarray) -> np.ndarray:
        clean_scan = scan.copy()

        # Bad readings = "nothing nearby"
        for i in range(len(clean_scan)):
            value = float(clean_scan[i])

            if not math.isfinite(value) or value <= 0.0:
                value = self.max_range_cm
            elif value > self.max_range_cm:
                value = self.max_range_cm

            clean_scan[i] = value / self.max_range_cm

        return clean_scan


class LidarTrainingData:
    def __init__(
        self,
        dataset_dir: Optional[Path],
        max_range_cm: float,
        val_ratio: float,
        seed: int,
        limit: int,
    ) -> None:
        if dataset_dir is None:
            dataset_dir = self.find_latest_dataset_dir()

        self.dataset_dir = dataset_dir
        self.max_range_cm = max_range_cm
        self.rows = self._load_rows(limit)
        self.num_beams = self._find_num_beams()
        self.scaler = self._build_scaler()

        self.train_rows, self.val_rows = self._split_rows(val_ratio, seed)

        self.train_dataset = LidarTrainingDataset(
            dataset_dir=self.dataset_dir,
            rows=self.train_rows,
            scaler=self.scaler,
            max_range_cm=self.max_range_cm,
            num_beams=self.num_beams,
        )
        self.val_dataset = LidarTrainingDataset(
            dataset_dir=self.dataset_dir,
            rows=self.val_rows,
            scaler=self.scaler,
            max_range_cm=self.max_range_cm,
            num_beams=self.num_beams,
        )

    @staticmethod
    def find_latest_dataset_dir() -> Path:
        candidates = []

        for path in DATA_DIR.iterdir():
            if path.is_dir() and path.name.startswith("lidar_samples_"):
                candidates.append(path)

        candidates.sort()

        if not candidates:
            raise FileNotFoundError("No lidar_samples_* dataset directories were found.")

        return candidates[-1]

    def make_dataloaders(
        self,
        batch_size: int,
        workers: int,
        pin_memory: bool,
    ) -> Tuple[DataLoader, DataLoader]:
        train_loader = DataLoader(
            self.train_dataset,
            batch_size=batch_size,
            shuffle=True,
            num_workers=workers,
            pin_memory=pin_memory,
        )
        val_loader = DataLoader(
            self.val_dataset,
            batch_size=batch_size,
            shuffle=False,
            num_workers=workers,
            pin_memory=pin_memory,
        )
        return train_loader, val_loader

    def _load_rows(self, limit: int) -> List[Dict[str, object]]:
        rows = []
        csv_path = self.dataset_dir / "samples.csv"

        with csv_path.open("r", newline="", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)

            for row_index, csv_row in enumerate(reader):
                if limit > 0 and row_index >= limit:
                    break

                sample_id_text = csv_row.get("sample_id", "").strip()
                if sample_id_text:
                    sample_id = int(sample_id_text)
                else:
                    sample_id = int(Path(csv_row["scan_file"]).stem)

                row = {
                    "sample_id": sample_id,
                    "scan_file": csv_row["scan_file"],
                    "x": float(csv_row["actual_x"]),
                    "z": float(csv_row["actual_z"]),
                    "yaw": float(csv_row["actual_yaw_rad"]),
                }
                rows.append(row)

        if not rows:
            raise ValueError(f"No samples found in {csv_path}")

        return rows

    def _find_num_beams(self) -> int:
        first_row = self.rows[0]
        scan_path = self.dataset_dir / str(first_row["scan_file"])
        first_scan = np.load(scan_path)
        return int(first_scan.size)

    def _build_scaler(self) -> Dict[str, float]:
        x_total = 0.0
        z_total = 0.0

        for row in self.rows:
            x_total += float(row["x"])
            z_total += float(row["z"])

        x_mean = x_total / len(self.rows)
        z_mean = z_total / len(self.rows)

        x_squared_error_total = 0.0
        z_squared_error_total = 0.0

        for row in self.rows:
            x_error = float(row["x"]) - x_mean
            z_error = float(row["z"]) - z_mean
            x_squared_error_total += x_error * x_error
            z_squared_error_total += z_error * z_error

        x_std = math.sqrt(x_squared_error_total / len(self.rows))
        z_std = math.sqrt(z_squared_error_total / len(self.rows))

        if x_std < 1e-6:
            x_std = 1.0
        if z_std < 1e-6:
            z_std = 1.0

        return {
            "x_mean": x_mean,
            "x_std": x_std,
            "z_mean": z_mean,
            "z_std": z_std,
        }

    def _split_rows(
        self,
        val_ratio: float,
        seed: int,
    ) -> Tuple[List[Dict[str, object]], List[Dict[str, object]]]:
        rows = list(self.rows)
        rng = random.Random(seed)
        rng.shuffle(rows)

        if len(rows) < 2:
            return rows, []

        val_count = int(len(rows) * val_ratio)
        if val_count < 1:
            val_count = 1
        if val_count >= len(rows):
            val_count = len(rows) - 1

        val_rows = rows[:val_count]
        train_rows = rows[val_count:]
        return train_rows, val_rows
