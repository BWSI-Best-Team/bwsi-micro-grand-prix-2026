from __future__ import annotations

import sys
from pathlib import Path

# Resolve both bundled third-party code and the local BWSI library at startup.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
STUDENT_ROOT = PROJECT_ROOT.parent
LIBRARY_DIR = STUDENT_ROOT / "library"
VENDOR_DIR = PROJECT_ROOT / "vendor" # bundled

sys.path.insert(0, str(VENDOR_DIR))
sys.path.insert(0, str(LIBRARY_DIR))

import racecar_core
from controller_app import GrandPrixController

rc = racecar_core.create_racecar()
controller = GrandPrixController(rc)

if __name__ == "__main__":
    rc.set_start_update(controller.start, controller.update, controller.update_slow)
    rc.go()
