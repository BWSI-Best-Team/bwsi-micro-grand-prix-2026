from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

from util.onnx_runtime_loader import load_onnxruntime_session


@dataclass(frozen=True)
class MLPose:
    x_m: float
    y_m: float
    yaw_rad: float
    confidence: float = float("nan")


class MLPoseEstimator:
    """
    Basic ONNX model loading for pose predictor
    Usage:
        estimator = MLPoseEstimator("models/lidar_locator_finetune.onnx")
        pose = estimator.predict(scan_cm)
    """

    def __init__(self, model_path: str | Path) -> None:
        self.model_path = Path(model_path)
        self.runtime = load_onnxruntime_session(self.model_path)
        self.session = self.runtime.session
        self.metadata = self._load_metadata(self.model_path)
        self.input_name = self.session.get_inputs()[0].name

        self.num_beams = int(self.metadata.get("num_beams", 720))
        self.num_scan_channels = int(self.metadata.get("num_scan_channels", 1))
        self.max_range_cm = float(self.metadata.get("max_range_cm", 1000.0))
        self.output_names = [output.name for output in self.session.get_outputs()]

    @property
    def active_providers(self) -> list[str]:
        return list(self.runtime.active_providers)

    def predict(self, scan_cm: np.ndarray) -> MLPose:
        scan_input = self._prepare_scan(scan_cm)
        outputs = self.session.run(None, {self.input_name: scan_input})
        outputs_by_name = dict(zip(self.output_names, outputs))

        pred_xz = outputs_by_name["pred_xz"][0]
        yaw_rad = self._decode_yaw(outputs_by_name)
        confidence = self._decode_confidence(outputs_by_name)
        return MLPose(
            x_m=float(pred_xz[0]),
            y_m=float(pred_xz[1]),
            yaw_rad=yaw_rad,
            confidence=confidence,
        )

    def _prepare_scan(self, scan_cm: np.ndarray) -> np.ndarray:
        scan = np.asarray(scan_cm, dtype=np.float32).reshape(-1)
        if scan.shape[0] != self.num_beams:
            raise ValueError(
                f"Expected {self.num_beams} LiDAR beams, got {scan.shape[0]}",
            )

        valid = np.isfinite(scan) & (scan > 0.0) & (scan <= self.max_range_cm)
        ranges = scan.copy()
        ranges[~valid] = self.max_range_cm
        ranges = np.clip(ranges, 0.0, self.max_range_cm) / self.max_range_cm

        if self.num_scan_channels == 1:
            model_scan = ranges[np.newaxis, :]
        elif self.num_scan_channels == 2:
            model_scan = np.stack([ranges, valid.astype(np.float32)], axis=0)
        else:
            raise ValueError(
                f"Unsupported num_scan_channels={self.num_scan_channels}; expected 1 or 2.",
            )

        return model_scan[np.newaxis, ...].astype(np.float32, copy=False)

    @staticmethod
    def _decode_yaw(outputs_by_name: dict[str, np.ndarray]) -> float:
        vec = np.asarray(outputs_by_name["yaw_vec"][0], dtype=np.float32)
        norm = float(np.linalg.norm(vec))
        if norm > 1e-6:
            vec = vec / norm
        return math.atan2(float(vec[0]), float(vec[1]))

    @staticmethod
    def _decode_confidence(outputs_by_name: dict[str, np.ndarray]) -> float:
        logits = np.asarray(outputs_by_name["heatmap_logits"][0], dtype=np.float64)
        logits = logits - np.max(logits)
        probs = np.exp(logits)
        denom = float(np.sum(probs))
        if denom <= 0.0 or not math.isfinite(denom):
            return float("nan")
        return float(np.max(probs / denom))

    @staticmethod
    def _load_metadata(model_path: Path) -> dict:
        metadata_path = model_path.with_suffix(".json")
        if metadata_path.exists():
            return json.loads(metadata_path.read_text())
        return {}
