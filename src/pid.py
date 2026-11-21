class PID:
    def __init__(self, kp, ki, kd, out_min=0.0, out_max=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.e_prev = None
        self.error = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def step(self, setpoint, measurement, dt):
        e = setpoint - measurement
        de = 0.0 if self.e_prev is None or dt <= 0 else (e - self.e_prev) / dt
        self.e_prev = e
        self.error = e
        self.integral += e * dt
        if self.ki > 0:
            low = self.out_min / self.ki
            high = self.out_max / self.ki
            self.integral = max(low, min(high, self.integral))
        self.p = self.kp * e
        self.i = self.ki * self.integral
        self.d = self.kd * de
        return max(self.out_min, min(self.out_max, self.p + self.i + self.d))
