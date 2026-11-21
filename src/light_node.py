from firmata import ADC, PWM
from pid import PID


class LightNode:
    MODES = ("off", "manual", "follow", "pid")

    def __init__(self, board, ldr_channel, led_pin, dark, bright, n=8):
        self.adc = ADC(board, ldr_channel)
        self.pwm = PWM(board, led_pin)
        self.dark = dark
        self.bright = bright
        self.n = n
        self.mode = "off"
        self.setpoint = 50.0
        self.gamma = 1.0
        self.pid = PID(1.0, 0.0, 0.0)
        self.raw = 0
        self.pct = 0.0
        self.u = 0.0

    def read_raw(self):
        return self.adc.read_avg(self.n)

    def pct_from_raw(self, raw):
        if self.bright == self.dark:
            return 0.0
        pct = (raw - self.dark) / (self.bright - self.dark) * 100
        return max(0.0, min(100.0, pct))

    def read_pct(self):
        self.raw = self.read_raw()
        self.pct = self.pct_from_raw(self.raw)
        return self.pct

    def set_brightness(self, pct):
        pct = max(0.0, min(100.0, pct))
        self.pwm.duty_u16(round(pct * 655.35))

    def brightness(self):
        return self.pwm.duty_u16() / 655.35

    def set_mode(self, mode):
        if mode not in self.MODES:
            raise ValueError(mode)
        if mode == "pid" and self.mode != "pid":
            self.pid.reset()
        self.mode = mode
        if mode == "off":
            self.set_brightness(0)

    def update(self, dt):
        self.read_pct()
        if self.mode == "off":
            self.set_brightness(0)
        elif self.mode == "follow":
            self.set_brightness(self.pct)
        elif self.mode == "pid":
            self.u = self.pid.step(self.setpoint, self.pct, dt)
            self.set_brightness(100 * (self.u / 100) ** self.gamma)
