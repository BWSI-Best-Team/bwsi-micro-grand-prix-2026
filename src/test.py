import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from util.library_finder import find_library
LIB = find_library(ROOT)
sys.path.insert(0, str(LIB))
sys.path.insert(0, str(LIB / "simulation"))

import numpy as np
import racecar_core

rc = racecar_core.create_racecar()

UNITY_TO_M = 0.10

def start():
    rc.drive.stop()
    print(">> Manual drive mode")

def update():
    rc.drive.set_max_speed(0.5)
    angle, speed = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    try:
        pose = rc.physics.get_simulator_not_official_pose()
        x_m = float(pose[0]) * UNITY_TO_M
        z_m = float(pose[2]) * UNITY_TO_M
        yaw = np.pi / 2.0 - float(pose[3])
        print(f"x: {x_m:.2f}  y: {z_m:.2f}  yaw: {yaw:.2f}")
    except AttributeError:
        print("simulator pose API not available")

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
