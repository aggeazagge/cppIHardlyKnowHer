import machine
import time
import vl53l0x

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

vanster = vl53l0x.VL53L0X(i2c)
vanster.set_address(0x30)


# Starta höger sensor
xshut_hoger.value(1)
time.sleep(0.1)

hoger = vl53l0x.VL53L0X(i2c)
hoger.set_address(0x31)


# Starta mitten sensor
xshut_mitten.value(1)
time.sleep(0.1)

mitten = vl53l0x.VL53L0X(i2c)


# Sätt kortast möjliga mättidsbudget och starta kontinuerlig mätning på alla
# tre sensorer -- las_avstand() blir då ett billigt registerläs istället för
# att trigga en hel ny mätning varje varv (se read()/start_continuous() i
# vl53l0x.py).
for sensor in (vanster, mitten, hoger):
    sensor.measurement_timing_budget = config.SENSOR_TIMING_BUDGET_US
    sensor.start_continuous()


def _clamp_reading(mm):
    """Clamp a raw VL53L0X reading to a usable range.

    Out-of-range measurements come back as 0 or a large number; treat both
    as 'nothing in the way' (DIST_MAX). Done here, before averaging, so a
    single bad raw reading can't drag the rolling average toward "wall
    right ahead."
    """
    if mm <= 0 or mm >= config.DIST_MAX:
        return config.DIST_MAX
    return mm


class _RollingAvg:
    """Fixed-size rolling mean over the last N clamped readings."""

    def __init__(self, window, initial):
        self._buf = [initial] * window

    def push(self, value):
        self._buf.pop(0)
        self._buf.append(value)
        return sum(self._buf) / len(self._buf)


_avg_vanster = _RollingAvg(config.SENSOR_AVG_SAMPLES, config.DIST_MAX)
_avg_mitten = _RollingAvg(config.SENSOR_AVG_SAMPLES, config.DIST_MAX)
_avg_hoger = _RollingAvg(config.SENSOR_AVG_SAMPLES, config.DIST_MAX)


# Funktion för att läsa alla sensorer (utjämnade med glidande medelvärde)
def las_avstand():
    avstand_vanster = _avg_vanster.push(_clamp_reading(vanster.read()))
    avstand_mitten = _avg_mitten.push(_clamp_reading(mitten.read()))
    avstand_hoger = _avg_hoger.push(_clamp_reading(hoger.read()))

    return avstand_vanster, avstand_mitten, avstand_hoger