import threading
import time
from collections import deque

import serial

ANALOG_MESSAGE = 0xE0
REPORT_ANALOG = 0xC0
SET_PIN_MODE = 0xF4
REPORT_VERSION = 0xF9
START_SYSEX = 0xF0
END_SYSEX = 0xF7
ANALOG_MAPPING_QUERY = 0x69
ANALOG_MAPPING_RESPONSE = 0x6A
CAPABILITY_QUERY = 0x6B
CAPABILITY_RESPONSE = 0x6C
EXTENDED_ANALOG = 0x6F
SAMPLING_INTERVAL = 0x7A
MODE_ANALOG = 2
MODE_PWM = 3


class Board:
    def __init__(self, port, baud=115200, sampling_ms=5, timeout=10.0):
        self.serial = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
        self.capabilities = {}
        self.analog_map = {}
        self.analog = {}
        self.history = {}
        self.version = None
        self._write_lock = threading.Lock()
        self._data_lock = threading.Lock()
        self._command = None
        self._data = []
        self._sysex = None
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self._handshake(timeout)
        self.set_sampling_interval(sampling_ms)

    def _handshake(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.write([REPORT_VERSION])
            self.write([START_SYSEX, CAPABILITY_QUERY, END_SYSEX])
            self.write([START_SYSEX, ANALOG_MAPPING_QUERY, END_SYSEX])
            retry = time.monotonic() + 1.0
            while time.monotonic() < retry:
                if self.capabilities and self.analog_map:
                    return
                time.sleep(0.02)
        port = self.serial.port
        self.close()
        raise RuntimeError(f"no Firmata reply on {port}, is StandardFirmata uploaded?")

    def _read_loop(self):
        while self._running:
            try:
                data = self.serial.read(256)
            except (serial.SerialException, OSError, TypeError):
                break
            for byte in data:
                self._feed(byte)

    def _feed(self, byte):
        if self._sysex is not None:
            if byte == END_SYSEX:
                self._handle_sysex(self._sysex)
                self._sysex = None
            else:
                self._sysex.append(byte)
            return
        if byte == START_SYSEX:
            self._sysex = []
            return
        if byte & 0x80:
            self._command = byte
            self._data = []
            return
        if self._command is None:
            return
        self._data.append(byte)
        if len(self._data) < 2:
            return
        if self._command & 0xF0 == ANALOG_MESSAGE:
            channel = self._command & 0x0F
            value = self._data[0] | (self._data[1] << 7)
            with self._data_lock:
                self.analog[channel] = value
                self.history.setdefault(channel, deque(maxlen=64)).append(value)
        elif self._command == REPORT_VERSION:
            self.version = f"{self._data[0]}.{self._data[1]}"
        self._data = []

    def _handle_sysex(self, data):
        if not data:
            return
        if data[0] == CAPABILITY_RESPONSE:
            pins = {}
            modes = {}
            i = 1
            while i < len(data):
                if data[i] == 0x7F:
                    pins[len(pins)] = modes
                    modes = {}
                    i += 1
                else:
                    if i + 1 < len(data):
                        modes[data[i]] = data[i + 1]
                    i += 2
            self.capabilities = pins
        elif data[0] == ANALOG_MAPPING_RESPONSE:
            self.analog_map = {ch: pin for pin, ch in enumerate(data[1:]) if ch != 0x7F}

    def samples(self, channel, n):
        with self._data_lock:
            history = self.history.get(channel)
            return list(history)[-n:] if history else []

    def write(self, data):
        with self._write_lock:
            self.serial.write(bytes(data))

    def set_sampling_interval(self, ms):
        self.write([START_SYSEX, SAMPLING_INTERVAL, ms & 0x7F, (ms >> 7) & 0x7F, END_SYSEX])

    def set_pin_mode(self, pin, mode):
        self.write([SET_PIN_MODE, pin, mode])

    def report_analog(self, channel, enable=True):
        self.write([REPORT_ANALOG | channel, 1 if enable else 0])

    def analog_write(self, pin, value):
        if pin < 16 and value < 0x4000:
            self.write([ANALOG_MESSAGE | pin, value & 0x7F, (value >> 7) & 0x7F])
        else:
            self.write([START_SYSEX, EXTENDED_ANALOG, pin, value & 0x7F,
                        (value >> 7) & 0x7F, (value >> 14) & 0x7F, END_SYSEX])

    def resolution(self, pin, mode, default):
        return self.capabilities.get(pin, {}).get(mode, default)

    def close(self):
        self._running = False
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        if self.serial.is_open:
            self.serial.close()


class ADC:
    def __init__(self, board, channel, timeout=2.0):
        self.board = board
        self.channel = channel
        self.pin = board.analog_map[channel]
        self.bits = board.resolution(self.pin, MODE_ANALOG, 10)
        self.max = (1 << self.bits) - 1
        board.set_pin_mode(self.pin, MODE_ANALOG)
        board.report_analog(channel)
        end = time.monotonic() + timeout
        while channel not in board.analog and time.monotonic() < end:
            time.sleep(0.01)

    def read(self):
        return self.board.analog.get(self.channel, 0)

    def read_u16(self):
        return round(self.read() * 65535 / self.max)

    def read_avg(self, n):
        values = self.board.samples(self.channel, n)
        if not values:
            return self.read_u16()
        return round(sum(values) / len(values) * 65535 / self.max)


class PWM:
    def __init__(self, board, pin, duty_u16=0):
        self.board = board
        self.pin = pin
        self.bits = board.resolution(pin, MODE_PWM, 8)
        self.max = (1 << self.bits) - 1
        self._duty = 0
        self._level = -1
        board.set_pin_mode(pin, MODE_PWM)
        self.duty_u16(duty_u16)

    def duty_u16(self, value=None):
        if value is None:
            return self._duty
        value = max(0, min(65535, int(value)))
        self._duty = value
        level = round(value * self.max / 65535)
        if level != self._level:
            self._level = level
            self.board.analog_write(self.pin, level)
        return value

    def deinit(self):
        self.duty_u16(0)
