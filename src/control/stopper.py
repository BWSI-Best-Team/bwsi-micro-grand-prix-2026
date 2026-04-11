# lab_g

from util.pid import PID
from util.constants import (
    GATE_STOP_M,
    STOPPER_FINE_TUNE_M,
    STOPPER_MAX_SPEED_MS as MAX_SPEED_MS,
)

STOP_CM = GATE_STOP_M * 100.0
FINE_TUNE_CM = STOPPER_FINE_TUNE_M * 100.0

distance_pid = PID(kp=0.003, ki=0.0, kd=0.001, output_min=-0.5, output_max=0.5)
speed_pid = PID(kp=0.009, ki=0.0001, kd=0.008, integral_min=-1.0, integral_max=1.0, output_min=-1.0, output_max=1.0)
fine_pid = PID(kp=0.001, ki=0.01, kd=0.005, output_min=-0.5, output_max=0.5)

class Stopper:
    def __init__(self):
        self.stopped = False
        self._fine_tuning = False

    def reset(self):
        self._reset_pids()
        self._fine_tuning = False
        self.stopped = False

    def update(self, dist_cm, v_fwd, dt):
        if dist_cm <= STOP_CM:
            self._reset_pids()
            self._fine_tuning = False
            self.stopped = True
            return 0.0

        remaining_cm = max(0.0, dist_cm - STOP_CM)

        if remaining_cm <= FINE_TUNE_CM:
            if not self._fine_tuning:
                distance_pid.reset()
                speed_pid.reset()
                fine_pid.reset()
                self._fine_tuning = True
            return max(-1.0, min(1.0, fine_pid.update(error=remaining_cm, dt=dt)))

        if self._fine_tuning:
            fine_pid.reset()
            self._fine_tuning = False

        error_cm = dist_cm
        goal = distance_pid.update(error=error_cm, dt=dt)
        ff = goal / MAX_SPEED_MS
        fb = speed_pid.update(error=goal - v_fwd, dt=dt)
        throttle = max(-1.0, min(1.0, ff + fb))
        return throttle

    def _reset_pids(self):
        distance_pid.reset()
        speed_pid.reset()
        fine_pid.reset()
