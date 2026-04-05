from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass
class ControllerConfig:
    update_slow_time_s: float = 0.5
    max_speed: float = 0.25
    blocked_distance_cm: float = 60.0
    print_log: bool = True
    show_visualizer: bool = False


def load_controller_config() -> ControllerConfig:
    config = ControllerConfig()
    config_path = Path(__file__).resolve().parents[1] / "controller_config.json"

    if not config_path.exists():
        return config

    with config_path.open("r", encoding="utf-8") as config_file:
        data = json.load(config_file)

    if not isinstance(data, dict):
        raise ValueError("controller_config.json must contain JSON.")

    _apply_overrides(config, data)
    return config


def _apply_overrides(config: ControllerConfig, overrides: dict[str, Any]) -> None:
    valid_keys = {
        "print_log",
        "show_visualizer",
    }

    unknown_keys = set(overrides) - valid_keys
    if unknown_keys:
        unknown_list = ", ".join(sorted(unknown_keys))
        raise ValueError(f"Unknown keys in config: {unknown_list}")

    for key, value in overrides.items():
        setattr(config, key, value)
