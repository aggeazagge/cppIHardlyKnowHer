"""
`vl53l4cx`
====================================================

Lean MicroPython driver for the VL53L4CX distance sensor (used for the left
and right sensors; the middle one is still a VL53L0X, see vl53l0x.py).

Ported from Adafruit's CircuitPython VL53L4CD driver, which follows ST's
VL53L4CD "ultra lite driver" (ULD):
https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CD

The VL53L4CX belongs to the same sensor family and reports the same model ID
(0xEBAA), so the ULD init sequence runs on it. Using the ULD caps the range at
roughly 1.3 m, which covers config.DIST_MAX.

Only continuous back-to-back ranging is supported, which is all
lidar_sensor.py needs. The public interface mirrors the VL53L0X driver:
start_continuous() / stop_continuous() / read_with_status() / set_address()
and the io_timeout_ms attribute.
"""
import utime
from micropython import const

_I2C_SLAVE_DEVICE_ADDRESS = const(0x0001)
_OSC_FREQUENCY = const(0x0006)
_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = const(0x0008)
_GPIO_HV_MUX_CTRL = const(0x0030)
_GPIO_TIO_HV_STATUS = const(0x0031)
_RANGE_CONFIG_A = const(0x005E)
_RANGE_CONFIG_B = const(0x0061)
_INTERMEASUREMENT_MS = const(0x006C)
_SYSTEM_INTERRUPT_CLEAR = const(0x0086)
_SYSTEM_START = const(0x0087)
_RESULT_RANGE_STATUS = const(0x0089)
_RESULT_DISTANCE = const(0x0096)
_FIRMWARE_SYSTEM_STATUS = const(0x00E5)
_IDENTIFICATION_MODEL_ID = const(0x010F)

_EXPECTED_MODEL_ID = const(0xEBAA)

# Range status returned by read_with_status() when the measurement is OK.
RANGE_VALID = const(0)
_RANGE_ERROR_OTHER = const(255)

# Raw RESULT_RANGE_STATUS (low 5 bits) -> ULD status. 0 = valid.
_STATUS_RTN = bytes((
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 255, 255, 255,
    255, 255, 10, 6, 255, 255, 11, 12,
))

# ULD default configuration, written to registers 0x2D..0x87 in one go.
# See the Adafruit/ST source for the per-register comments.
_DEFAULT_CONFIG = bytes((
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08,  # 0x2D..0x34
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,  # 0x35..0x3C
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,  # 0x3D..0x44
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21,  # 0x45..0x4C
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,  # 0x4D..0x54
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,  # 0x55..0x5C
    0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05, 0x00,  # 0x5D..0x64
    0xA0, 0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00,  # 0x65..0x6C
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,  # 0x6D..0x74
    0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00,  # 0x75..0x7C
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,  # 0x7D..0x84
    0x01, 0x00, 0x00,                                # 0x85..0x87
))

# Upper bound for the waits during init, before io_timeout_ms is set.
_INIT_TIMEOUT_MS = const(1000)


class TimeoutCheck:
    """Very simple sensor timeout checker class (same as in vl53l0x.py)."""
    def __init__(self, timeout_ms):
        self._start = utime.ticks_ms()
        self._timeout_ms = timeout_ms

    def check(self):
        """Checks for sensor timeout."""
        if (
            self._timeout_ms > 0
            and (utime.ticks_diff(utime.ticks_ms(), self._start)) >= self._timeout_ms
        ):
            raise RuntimeError("Timeout waiting for VL53L4CX!")


class VL53L4CX:
    """Driver for the VL53L4CX distance sensor (continuous ranging only)."""

    def __init__(self, i2c, address=0x29, io_timeout_ms=0):
        self._i2c = i2c
        self._addr = address
        self._continuous_mode = False
        self.io_timeout_ms = io_timeout_ms

        self._wait_for_boot()
        self._check_for_vl53l4cx()
        self._write(0x002D, _DEFAULT_CONFIG)

        # VHV warm-up measurement, as in the ULD's SensorInit().
        self._write_u8(_SYSTEM_START, 0x40)
        self._wait_data_ready(_INIT_TIMEOUT_MS)
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write_u8(_SYSTEM_START, 0x00)

        self._write_u8(_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
        self._write_u8(0x000B, 0x00)
        self._write_u16(0x0024, 0x0500)
        # Inter-measurement 0 = continuous back-to-back ranging.
        self._write(_INTERMEASUREMENT_MS, bytes(4))
        self.set_timing(50)

    # --- low-level I2C (16-bit register addresses) -------------------------

    def _read(self, reg, n):
        return self._i2c.readfrom_mem(self._addr, reg, n, addrsize=16)

    def _read_u8(self, reg):
        return self._read(reg, 1)[0]

    def _read_u16(self, reg):
        buf = self._read(reg, 2)
        return (buf[0] << 8) | buf[1]

    def _write(self, reg, data):
        self._i2c.writeto_mem(self._addr, reg, data, addrsize=16)

    def _write_u8(self, reg, val):
        self._write(reg, bytes((val & 0xFF,)))

    def _write_u16(self, reg, val):
        self._write(reg, bytes(((val >> 8) & 0xFF, val & 0xFF)))

    # --- init helpers --------------------------------------------------------

    def _wait_for_boot(self):
        timechecker = TimeoutCheck(_INIT_TIMEOUT_MS)
        while True:
            try:
                if self._read_u8(_FIRMWARE_SYSTEM_STATUS) == 0x03:
                    return
            except OSError:
                pass  # sensor may NACK while still booting
            timechecker.check()
            utime.sleep_ms(1)

    def _check_for_vl53l4cx(self):
        model_id = self._read_u16(_IDENTIFICATION_MODEL_ID)
        if model_id != _EXPECTED_MODEL_ID:
            raise RuntimeError(
                "VL53L4CX at 0x%02X: unexpected model ID 0x%04X (expected 0x%04X)"
                % (self._addr, model_id, _EXPECTED_MODEL_ID)
            )
        print("VL53L4CX Sensor Found.")

    def _data_ready(self):
        # Interrupt polarity: bit 4 of GPIO_HV_MUX_CTRL set = active low.
        polarity = 0 if self._read_u8(_GPIO_HV_MUX_CTRL) & 0x10 else 1
        return (self._read_u8(_GPIO_TIO_HV_STATUS) & 0x01) == polarity

    def _wait_data_ready(self, timeout_ms):
        timechecker = TimeoutCheck(timeout_ms)
        while not self._data_ready():
            timechecker.check()

    # --- public API ----------------------------------------------------------

    def set_timing(self, budget_ms):
        """Set the ranging timing budget in ms (10..200). Must not be ranging.
        Port of the ULD's VL53L4CD_SetRangeTiming() for continuous mode.
        """
        if self._continuous_mode:
            raise RuntimeError("Stop ranging before changing the timing budget.")
        if not 10 <= budget_ms <= 200:
            raise ValueError("Timing budget must be 10..200 ms.")
        osc_freq = self._read_u16(_OSC_FREQUENCY)
        if osc_freq == 0:
            raise RuntimeError("VL53L4CX oscillator frequency is 0.")

        macro_period_us = int(2304 * (1073741824.0 / osc_freq)) >> 6
        timing_budget_us = (budget_ms * 1000 - 2500) << 12

        for reg, mult in ((_RANGE_CONFIG_A, 16), (_RANGE_CONFIG_B, 12)):
            tmp = (macro_period_us * mult) >> 6
            ls_byte = int((timing_budget_us + (tmp >> 1)) / tmp) - 1
            ms_byte = 0
            while ls_byte >> 8:
                ls_byte >>= 1
                ms_byte += 1
            self._write_u16(reg, (ms_byte << 8) | (ls_byte & 0xFF))

    def start_continuous(self):
        """Start back-to-back continuous ranging."""
        self._write_u8(_SYSTEM_START, 0x21)
        # Wait for (and discard) the first result, like the ULD does.
        self._wait_data_ready(self.io_timeout_ms or _INIT_TIMEOUT_MS)
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._continuous_mode = True

    def stop_continuous(self):
        """Stop ranging."""
        self._write_u8(_SYSTEM_START, 0x00)
        self._continuous_mode = False

    def read_with_status(self):
        """Wait for the next continuous-mode result and return
        ``(range_mm, status)``; status RANGE_VALID (0) means the sensor
        considers the measurement valid.

        Raises RuntimeError if no result arrives within io_timeout_ms.
        """
        self._wait_data_ready(self.io_timeout_ms)
        raw = self._read_u8(_RESULT_RANGE_STATUS) & 0x1F
        range_mm = self._read_u16(_RESULT_DISTANCE)
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        status = _STATUS_RTN[raw] if raw < len(_STATUS_RTN) else _RANGE_ERROR_OTHER
        return range_mm, status

    def set_address(self, new_address):
        """Change the sensor's I2C address. All other sensors still on the
        default address must be held in shutdown (XSHUT low) while doing this.
        """
        self._write_u8(_I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F)
        self._addr = new_address
