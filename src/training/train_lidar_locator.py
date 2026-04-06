import json
from pathlib import Path
import random

import numpy as np
import torch
from torch import nn

from lidar_data import LidarTrainingData


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR = PROJECT_ROOT / "models"


# Edit these if you want different training settings.
DATASET_DIR = None
EPOCHS = 20
BATCH_SIZE = 256
LEARNING_RATE = 1e-3
VAL_RATIO = 0.15
MAX_RANGE_CM = 1000.0
SEED = 13371337
LIMIT = 0
WORKERS = 0
OUTPUT_PATH = MODELS_DIR / "lidar_locator.pt"


class LidarLocatorCNN(nn.Module):
    def __init__(self, num_beams: int) -> None:
        super().__init__()

        self.conv1 = nn.Conv1d(1, 32, kernel_size=7, padding=3)
        self.pool1 = nn.MaxPool1d(2)

        self.conv2 = nn.Conv1d(32, 64, kernel_size=5, padding=2)
        self.pool2 = nn.MaxPool1d(2)

        self.conv3 = nn.Conv1d(64, 128, kernel_size=5, padding=2)
        self.pool3 = nn.MaxPool1d(2)

        self.conv4 = nn.Conv1d(128, 128, kernel_size=3, padding=1)
        self.final_pool = nn.AdaptiveAvgPool1d(16)

        self.fc1 = nn.Linear(128 * 16, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 4)

        self.num_beams = num_beams

    def forward(self, scan: torch.Tensor) -> torch.Tensor:
        scan = torch.relu(self.conv1(scan))
        scan = self.pool1(scan)

        scan = torch.relu(self.conv2(scan))
        scan = self.pool2(scan)

        scan = torch.relu(self.conv3(scan))
        scan = self.pool3(scan)

        scan = torch.relu(self.conv4(scan))
        scan = self.final_pool(scan)

        scan = torch.flatten(scan, start_dim=1)
        scan = torch.relu(self.fc1(scan))
        scan = torch.relu(self.fc2(scan))
        scan = self.fc3(scan)
        return scan


class LidarLocatorTrainer:
    def __init__(
        self,
        training_data: LidarTrainingData,
        learning_rate: float,
        output_path: Path,
        batch_size: int,
        workers: int,
    ) -> None:
        self.training_data = training_data
        self.output_path = output_path

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
            pin_memory = True
        else:
            self.device = torch.device("cpu")
            pin_memory = False

        self.train_loader, self.val_loader = training_data.make_dataloaders(
            batch_size=batch_size,
            workers=workers,
            pin_memory=pin_memory,
        )

        self.model = LidarLocatorCNN(training_data.num_beams).to(self.device)
        self.optimizer = torch.optim.AdamW(self.model.parameters(), lr=learning_rate)
        self.loss_fn = nn.MSELoss()
        self.best_val_loss = float("inf")

    def run(self, epochs: int) -> None:
        print(">> training lidar locator")
        print(f"   dataset: {self.training_data.dataset_dir}")
        print(f"   samples: {len(self.training_data.rows)}")
        print(f"   num_beams: {self.training_data.num_beams}")
        print(f"   train samples: {len(self.training_data.train_rows)}")
        print(f"   val samples: {len(self.training_data.val_rows)}")
        print(f"   device: {self.device}")
        print(f"   output: {self.output_path}")

        for epoch in range(1, epochs + 1):
            train_loss = self._train_one_epoch()
            val_loss = self._run_validation()

            print(
                f"[epoch {epoch:03d}] "
                f"train_loss={train_loss:.6f} "
                f"val_loss={val_loss:.6f}"
            )

            if val_loss < self.best_val_loss:
                self.best_val_loss = val_loss
                self._save_model()
                print(f"   saved: {self.output_path}")

    def _train_one_epoch(self) -> float:
        self.model.train()
        total_loss = 0.0
        total_items = 0

        for scans, targets in self.train_loader:
            scans = scans.to(self.device)
            targets = targets.to(self.device)

            self.optimizer.zero_grad()
            predictions = self.model(scans)
            loss = self.loss_fn(predictions, targets)
            loss.backward()
            self.optimizer.step()

            batch_size = scans.shape[0]
            total_loss += float(loss.item()) * batch_size
            total_items += batch_size

        if total_items == 0:
            return 0.0

        return total_loss / total_items

    def _run_validation(self) -> float:
        if len(self.training_data.val_rows) == 0:
            return 0.0

        self.model.eval()
        total_loss = 0.0
        total_items = 0

        with torch.no_grad():
            for scans, targets in self.val_loader:
                scans = scans.to(self.device)
                targets = targets.to(self.device)

                predictions = self.model(scans)
                loss = self.loss_fn(predictions, targets)

                batch_size = scans.shape[0]
                total_loss += float(loss.item()) * batch_size
                total_items += batch_size

        if total_items == 0:
            return 0.0

        return total_loss / total_items

    def _save_model(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        torch.save(
            {
                "model_state_dict": self.model.state_dict(),
                "num_beams": self.model.num_beams,
                "scaler": self.training_data.scaler,
                "max_range_cm": self.training_data.max_range_cm,
                "dataset_dir": str(self.training_data.dataset_dir),
                "train_count": len(self.training_data.train_rows),
                "val_count": len(self.training_data.val_rows),
            },
            self.output_path,
        )

        metadata_path = self.output_path.with_suffix(".json")
        metadata_path.write_text(
            json.dumps(
                {
                    "num_beams": self.model.num_beams,
                    "scaler": self.training_data.scaler,
                    "max_range_cm": self.training_data.max_range_cm,
                    "dataset_dir": str(self.training_data.dataset_dir),
                    "train_count": len(self.training_data.train_rows),
                    "val_count": len(self.training_data.val_rows),
                },
                indent=2,
            )
        )


def main() -> None:
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)

    if DATASET_DIR:
        dataset_dir = Path(DATASET_DIR)
    else:
        dataset_dir = None

    training_data = LidarTrainingData(
        dataset_dir=dataset_dir,
        max_range_cm=MAX_RANGE_CM,
        val_ratio=VAL_RATIO,
        seed=SEED,
        limit=LIMIT,
    )

    trainer = LidarLocatorTrainer(
        training_data=training_data,
        learning_rate=LEARNING_RATE,
        output_path=Path(OUTPUT_PATH),
        batch_size=BATCH_SIZE,
        workers=WORKERS,
    )
    trainer.run(EPOCHS)


if __name__ == "__main__":
    main()
