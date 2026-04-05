from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path
import sys
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
VENDOR_DIR = PROJECT_ROOT / "vendor"
GPU_VENDOR_DIR = VENDOR_DIR / "onnxruntime_gpu_py39"
CPU_VENDOR_DIR = VENDOR_DIR / "onnxruntime_cpu_py39"


@dataclass
class OnnxRuntimeSession:
    ort: Any
    session: Any
    package_dir: Path
    requested_providers: list[str]
    active_providers: list[str]


def _clear_onnxruntime_modules() -> None:
    names = [
        name
        for name in sys.modules
        if name == "onnxruntime" or name.startswith("onnxruntime.")
    ]
    for name in names:
        del sys.modules[name]


def _remove_vendor_paths() -> None:
    for path in (str(GPU_VENDOR_DIR), str(CPU_VENDOR_DIR)):
        while path in sys.path:
            sys.path.remove(path)


def _import_onnxruntime(package_dir: Path) -> Any:
    _clear_onnxruntime_modules()
    _remove_vendor_paths()
    sys.path.insert(0, str(package_dir))
    return importlib.import_module("onnxruntime")


def _create_session(ort: Any, model_path: str | Path, providers: list[str]) -> Any:
    return ort.InferenceSession(str(model_path), providers=providers)


def load_onnxruntime_session(model_path: str | Path) -> OnnxRuntimeSession:
    """
    Loads ONNX Runtime from vendor/ with GPU preferred and CPU fallback.

    Call this before importing onnxruntime anywhere else in the process.
    """
    model_path = Path(model_path)
    gpu_error: Exception | None = None

    try:
        ort = _import_onnxruntime(GPU_VENDOR_DIR)
        requested = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        session = _create_session(ort, model_path, requested)
        return OnnxRuntimeSession(
            ort=ort,
            session=session,
            package_dir=GPU_VENDOR_DIR,
            requested_providers=requested,
            active_providers=list(session.get_providers()),
        )
    except Exception as exc:
        gpu_error = exc

    try:
        ort = _import_onnxruntime(CPU_VENDOR_DIR)
        requested = ["CPUExecutionProvider"]
        session = _create_session(ort, model_path, requested)
        return OnnxRuntimeSession(
            ort=ort,
            session=session,
            package_dir=CPU_VENDOR_DIR,
            requested_providers=requested,
            active_providers=list(session.get_providers()),
        )
    except Exception as exc:
        raise RuntimeError(
            "Failed to create an ONNX Runtime session from both vendored packages. "
            f"GPU error was: {gpu_error!r}. CPU error was: {exc!r}."
        ) from exc
