import time

class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, integral_min=None, integral_max=None, output_min=None, output_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._last_error = None
        self._last_time = None

    def update(self, error=None, setpoint=None, measurement=None, dt=None):
        now = time.time()
        if dt is None:
            if self._last_time is not None:
                dt = now - self._last_time
            else:
                dt = 1.0
        self._last_time = now

        if error is None:
            if setpoint is not None and measurement is not None:
                error = measurement - setpoint
            else:
                return 0.0

        # P
        p_term = self.kp * error

        # I
        self._integral += error * dt
        if self.integral_min is not None:
            self._integral = max(self.integral_min, self._integral)
        if self.integral_max is not None:
            self._integral = min(self.integral_max, self._integral)
        i_term = self.ki * self._integral

        # D
        if self._last_error is not None and dt > 0:
            d_term = self.kd * (error - self._last_error) / dt
        else:
            d_term = 0.0
        self._last_error = error

        out = p_term + i_term + d_term
        if self.output_min is not None:
            out = max(self.output_min, out)
        if self.output_max is not None:
            out = min(self.output_max, out)
        return out

    def reset(self):
        self._integral = 0.0
        self._last_error = None
        self._last_time = None
