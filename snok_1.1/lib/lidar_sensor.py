import machine
import time
import vl53l0x
import vl53l4cx

import config


# I2C
i2c = machine.I2C(
    0,
    sda=machine.Pin(11),
    scl=machine.Pin(12),
    freq=400000
)


# XSHUT-pinnar
xshut_vanster = machine.Pin(7, machine.Pin.OUT)
xshut_mitten = machine.Pin(6, machine.Pin.OUT)
xshut_hoger = machine.Pin(8, machine.Pin.OUT)


# Stäng av alla sensorer
xshut_vanster.value(0)
xshut_mitten.value(0)
xshut_hoger.value(0)
time.sleep(0.1)


# Starta vänster sensor
xshut_vanster.value(1)
time.sleep(0.1)

vanster = vl53l4cx.VL53L4CX(i2c)
vanster.set_address(0x30)


# Starta höger sensor
xshut_hoger.value(1)
time.sleep(0.1)

hoger = vl53l4cx.VL53L4CX(i2c)
hoger.set_address(0x31)


# Starta mitten sensor
xshut_mitten.value(1)
time.sleep(0.1)

mitten = vl53l0x.VL53L0X(i2c)


# Sätt mättidsbudget och starta kontinuerlig mätning på alla tre sensorer --
# las_avstand() blir då ett billigt registerläs istället för att trigga en hel
# ny mätning varje varv. Mitten är en VL53L0X, sidorna är VL53L4CX (olika
# drivrutiner: vl53l0x.py / vl53l4cx.py).
mitten.measurement_timing_budget = config.SENSOR_TIMING_BUDGET_US
for sensor in (vanster, hoger):
    sensor.set_timing(config.SIDE_SENSOR_TIMING_BUDGET_MS)
for sensor in (vanster, mitten, hoger):
    sensor.start_continuous()
    # Timeout sätts först efter init, så att init beter sig exakt som förut.
    # Utan den hänger read() för evigt om sensorn slutar leverera data.
    sensor.io_timeout_ms = config.SENSOR_READ_TIMEOUT_MS


def _clamp_reading(mm):
    """Clamp a raw sensor reading to a usable range.

    Out-of-range measurements come back as 0 or a large number; treat both
    as 'nothing in the way' (DIST_MAX). Done here, before averaging, so a
    single bad raw reading can't drag the rolling average toward "wall
    right ahead."
    """
    if mm <= 0 or mm >= config.DIST_MAX:
        return config.DIST_MAX
    return mm


# Range status meaning "measurement OK" differs per driver.
_VL53L0X_RANGE_VALID = 11
_VL53L4CX_RANGE_VALID = vl53l4cx.RANGE_VALID


class _RollingMedian:
    """Fixed-size rolling median over the last N clamped readings.

    Unlike a mean, a single spiky reading is dropped entirely instead of
    dragging the result toward it.
    """

    def __init__(self, window, initial):
        self._buf = [initial] * window

    def push(self, value):
        self._buf.pop(0)
        self._buf.append(value)
        return self.value()

    def value(self):
        return sorted(self._buf)[len(self._buf) // 2]


class _Channel:
    """One sensor + its filter. Never hangs; only raises if the sensor is dead.

    - Valid reading -> clamped and pushed into the median filter.
    - Invalid status (sigma/phase fail etc.) -> DIST_MAX if it read out of
      range, otherwise ignored (last filtered value is held).
    - Timeout / I2C error -> last value held. After a few in a row the
      sensor's continuous mode is restarted; after many, RuntimeError so
      main.run() brakes instead of driving blind.
    """

    def __init__(self, name, sensor, valid_status):
        self.name = name
        self.sensor = sensor
        self._valid_status = valid_status
        self._filter = _RollingMedian(config.SENSOR_AVG_SAMPLES, config.DIST_MAX)
        self._fails = 0

    def read(self):
        try:
            mm, status = self.sensor.read_with_status()
        except (RuntimeError, OSError) as e:
            self._fails += 1
            print("sensor", self.name, "read failed", self._fails, e)
            if self._fails == config.SENSOR_RESTART_AFTER_FAILS:
                self._restart()
            if self._fails >= config.SENSOR_MAX_FAILS:
                raise RuntimeError("sensor " + self.name + " dead")
            return self._filter.value()

        self._fails = 0
        if status == self._valid_status and mm > 0:
            return self._filter.push(_clamp_reading(mm))
        if mm >= config.DIST_MAX:
            return self._filter.push(config.DIST_MAX)
        return self._filter.value()

    def _restart(self):
        print("sensor", self.name, "restarting continuous mode")
        try:
            self.sensor.stop_continuous()
            self.sensor.start_continuous()
        except (RuntimeError, OSError) as e:
            print("sensor", self.name, "restart failed", e)


_kanal_vanster = _Channel("vanster", vanster, _VL53L4CX_RANGE_VALID)
_kanal_mitten = _Channel("mitten", mitten, _VL53L0X_RANGE_VALID)
_kanal_hoger = _Channel("hoger", hoger, _VL53L4CX_RANGE_VALID)


# Funktion för att läsa alla sensorer (utjämnade med glidande median)
def las_avstand():
    avstand_vanster = _kanal_vanster.read()
    avstand_mitten = _kanal_mitten.read()
    avstand_hoger = _kanal_hoger.read()

    return avstand_vanster, avstand_mitten, avstand_hoger