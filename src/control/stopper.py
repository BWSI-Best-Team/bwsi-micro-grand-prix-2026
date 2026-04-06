# lab_g

from util.pid import PID
from util.constants import STOPPER_TARGET_CM as TARGET_CM, STOPPER_MAX_SPEED_MS as MAX_SPEED_MS  # noqa: keep aliases

distance_pid = PID(kp=0.02, ki=0.0001, kd=0.002, output_min=-0.5, output_max=0.5)
speed_pid = PID(kp=2.0, ki=0.001, kd=0.02, integral_min=-1.0, integral_max=1.0, output_min=-1.0, output_max=1.0)
fine_pid = PID(kp=0.02, ki=0.0, kd=0.0001, output_min=-0.12, output_max=0.12)

class Stopper:
    def __init__(self):
        self.fine_tuning = False
        self.stopped = False

    def reset(self):
        distance_pid.reset()
        speed_pid.reset()
        fine_pid.reset()
        self.fine_tuning = False
        self.stopped = False

    def update(self, dist_cm, v_fwd, dt):
        error_cm = dist_cm - TARGET_CM

        if dist_cm <= 0:
            self.fine_tuning = False
            self._reset_pids()
            self.stopped = True
            return 0.0

        if self.fine_tuning:
            return self._compute_fine_tune(error_cm, dt)
        else:
            return self._compute_approach(error_cm, v_fwd, dt)

    def _compute_approach(self, error_cm, v_fwd, dt):
        goal = distance_pid.update(error=error_cm, dt=dt)
        ff = goal / MAX_SPEED_MS
        fb = speed_pid.update(error=goal - v_fwd, dt=dt)
        throttle = max(-1.0, min(1.0, ff + fb))

        if abs(error_cm) < 0.5 and abs(v_fwd) < 0.05:
            self.fine_tuning = True
            self._reset_pids()
            print("[Stopper] fine-tune mode")
            return 0.0

        return throttle

    def _compute_fine_tune(self, error_cm, dt):
        if abs(error_cm) > 3:
            self.fine_tuning = False
            self._reset_pids()
            return 0.0

        throttle = fine_pid.update(error=error_cm, dt=dt)

        if abs(error_cm) < 0.5 and abs(throttle) < 0.01:
            self.stopped = True
            return 0.0

        return throttle

    def _reset_pids(self):
        distance_pid.reset()
        speed_pid.reset()
