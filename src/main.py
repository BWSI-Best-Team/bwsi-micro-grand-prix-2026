from __future__ import annotations

import sys
from pathlib import Path

# Resolve both bundled third-party code and the local BWSI library at startup.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
VENDOR_DIR = PROJECT_ROOT / "vendor" # bundled

# Find the BWSI library
def _find_library(start: Path) -> Path:
    for candidate in [
        start.parent / "library",                                     # standard student layout
        start.parent.parent / "racecar-neo-installer" / "racecar-student" / "library",
        start.parent / "racecar-neo-installer" / "racecar-student" / "library",
    ]:
        if (candidate / "racecar_core.py").exists():
            return candidate
    raise FileNotFoundError("Could not find BWSI library directory")

LIBRARY_DIR = _find_library(PROJECT_ROOT)

sys.path.insert(0, str(VENDOR_DIR))
sys.path.insert(0, str(LIBRARY_DIR))
sys.path.insert(0, str(LIBRARY_DIR / "simulation"))
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import racecar_core
from controller_app import GrandPrixController

rc = racecar_core.create_racecar()
controller = GrandPrixController(rc)

if __name__ == "__main__":
    rc.set_start_update(controller.start, controller.update, controller.update_slow)
    rc.go()
