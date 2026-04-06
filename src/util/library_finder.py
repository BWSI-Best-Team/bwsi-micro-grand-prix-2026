""" Find BWSI racecar library directory """
from pathlib import Path


def find_library(project_root):
    for candidate in [
        project_root.parent / "library",
        project_root.parent.parent / "racecar-neo-installer" / "racecar-student" / "library",
        project_root.parent / "racecar-neo-installer" / "racecar-student" / "library",
    ]:
        if (candidate / "racecar_core.py").exists():
            return candidate
    raise FileNotFoundError("Could not find BWSI library directory")
