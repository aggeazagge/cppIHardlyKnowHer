# Autonomous tracked (excavator) chassis firmware

MicroPython firmware for a tracked, skid-steer chassis (two independent
tracks, no steering servo). The vehicle drives itself down a walled track
roughly 1 m wide, using three time-of-flight distance sensors (a VL53L0X in
the middle, VL53L4CX sensors on the left and right) to stay
centered between the walls. Unlike the Ackermann-steered `optimization\`
variant this is based on, this chassis has no steering angle: it always
drives both tracks at full `CRUISE_SPEED`, and steers purely by slowing the
inside track for a turn (e.g. the left track slows for a left turn). There is
no reverse -- dead ends and sharp corners are handled by committing harder to
the turn (see `compute()`) rather than backing out.

This README exists to help you (re-)learn how the project fits together --
it explains not just *what* each piece does but *why* it's built that way.

## Hardware / wiring

The board is an ESP32-family MicroPython target (see `.vscode/settings.json`,
which points the ESP-IDF extension at the toolchain). All pin numbers below
come from `config.py` and `lib/lidar_sensor.py`.

| Signal | Pin | Notes |
|---|---|---|
| I2C SDA | GPIO11 | shared bus -- sensors + Motoron |
| I2C SCL | GPIO12 | 400 kHz |
| Left ToF sensor (VL53L4CX) XSHUT | GPIO8 | reassigned to I2C address `0x30` |
| Middle ToF sensor (VL53L0X) XSHUT | GPIO6 | left at the power-on default, `0x29` |
| Right ToF sensor (VL53L4CX) XSHUT | GPIO7 | reassigned to I2C address `0x31` |
| Start-module input | GPIO17 | active level / pull configurable, see below |

The Motoron motor controller sits on the same I2C bus at address `0x10`
(`config.MOTORON_ADDRESS = 16`), driving the left track on channel 1
(`config.MOTOR_CHANNEL_LEFT`) and the right track on channel 2
(`config.MOTOR_CHANNEL_RIGHT`).

**Why the sensors need a startup sequence:** all three sensors (VL53L0X and
VL53L4CX alike) power up at the same default I2C address, `0x29`. `lib/lidar_sensor.py` handles this by
holding all three in reset via their XSHUT pins, then releasing them one at a
time and calling `set_address()` before releasing the next -- that's how the
left, middle, and right sensors end up addressable independently on one bus.
This has to happen *before* the Motoron is initialized in `main.py`, since
`main.py` reuses the I2C bus object that `lidar_sensor.py` already created.

The start-pin active level/pull (`START_ACTIVE_LOW`, `START_PIN_PULL`) is a
bench-calibration value in `config.py` -- it depends on how the specific
start module is physically wired.

## How it runs

MicroPython has a fixed boot convention: on every power-up (or wake from
deep sleep), it runs `boot.py` first, then `main.py`. In this project:

- **`boot.py`** is currently just the stock stub MicroPython ships with
  (commented-out debug/WebREPL setup) -- it does no project-specific work.
- **`main.py`** is the real entry point. Importing it has side effects: it
  sets up the Motoron controller for both tracks, and calls `run()`
  unconditionally at the bottom of the file. `run()` is an infinite loop --
  it *is* the program, not just a function you call from somewhere else.

The loop is gated by the start pin: it only drives when `start_requested()`
reads the configured "go" level; otherwise it brakes both tracks and waits.

> Note: nothing in this repo documents the flashing/deployment step itself
> (no `requirements.txt`, no `mpremote`/`ampy` scripts). In practice, getting
> this running means copying `boot.py`, `config.py`, `main.py`, and `lib/`
> onto the device's filesystem with a MicroPython tool and power-cycling the
> board -- but treat that as an inferred workflow, not something confirmed by
> the code here.

## Architecture: the control loop

The whole "brain" of the vehicle is the `run()` loop in `main.py`, which every
~10 ms (`LOOP_DELAY_MS`) does: read sensors -> compute left/right track speed
-> apply them -> repeat.

### Why three sensors, and why they're used differently

The left and right sensors are mounted **splayed outward** (angled away from
straight-ahead), not straight forward. On a track only ~1 m wide, that means
they're always going to see a nearby wall -- they're not "obstacle" sensors,
they're a **wall-centering signal**: if the right sensor sees more open space
than the left, the vehicle is closer to the left wall than the right, and
should steer right. The middle sensor is read every loop and logged, but this
variant doesn't use it for anything -- there's no distance-based speed ramp
here, since the vehicle always runs at full speed (see below).

### `compute(dl, dr)` -- the control law

Given left/right distances in mm, this function (in `main.py`) returns a
speed for each track:

1. **Steering fraction** starts from `dr - dl`: positive means more room on
   the right, so steer right. This difference is clamped to `±STEER_SPAN_MM`
   and turned into a fraction from -1 (full left) to +1 (full right) -- the
   same wall-centering signal the Ackermann variant uses, just not turned
   into a servo angle.
2. **Differential speed**: both tracks default to `CRUISE_SPEED`. Only the
   inside track for the current turn direction is slowed, proportional to
   how hard the turn is: `left_speed = CRUISE_SPEED * (1 - TURN_SLOWDOWN *
   max(0, -frac))`, and symmetrically for `right_speed` with `max(0, frac)`.
   On a straight (`frac == 0`) both tracks run at full `CRUISE_SPEED`; on a
   turn, only the inside track's speed drops, the outside track never slows
   down. This is deliberately different from the Ackermann car's behavior of
   throttling the *whole vehicle* down for wall-centering or corners -- a
   tracked chassis steers by speed differential, so there's no need to slow
   the outside track at all.

### Why there's no reverse

If the vehicle ends up nose-to-wall, it can't back out -- it can only keep
"steering into" the situation by widening the speed differential between the
tracks. Unlike the Ackermann car, a full pivot turn (one track forward, one
stopped or reversed) is mechanically possible on this chassis and could be
added later if `TURN_SLOWDOWN` alone isn't enough to clear a sharp corner --
that's a deliberate simplification left out of this version, not a hardware
limitation.

### Sensor smoothing

`lib/lidar_sensor.py` is the same as in the Ackermann variant (apart from the
XSHUT pin numbers) and exposes `las_avstand()` ("read distance"), which reads
all three sensors and returns smoothed values:

- `_clamp_reading()` treats any raw reading of `0` or `>= DIST_MAX` as "open"
  (`DIST_MAX`) *before* it's filtered, so one garbage raw sample can't drag
  the value toward "wall right ahead."
- Each reading's **range status** is checked (different "valid" codes for the
  VL53L0X and the VL53L4CX). Invalid measurements are ignored and the last
  filtered value is held instead.
- A small rolling median (`_RollingMedian`, window size `SENSOR_AVG_SAMPLES`,
  keep it odd) drops single spiky readings entirely, so the vehicle doesn't
  jerk its heading based on one bad sample.
- Each sensor runs in **continuous ranging mode** (`SENSOR_TIMING_BUDGET_US`
  for the middle VL53L0X, `SIDE_SENSOR_TIMING_BUDGET_MS` for the VL53L4CX
  sides, both ~33 ms), so each `las_avstand()` call is a cheap register read
  instead of triggering a whole new measurement.
- Reads never hang: if a sensor delivers no data within
  `SENSOR_READ_TIMEOUT_MS`, the last value is held. After
  `SENSOR_RESTART_AFTER_FAILS` failures in a row the sensor is restarted, and
  after `SENSOR_MAX_FAILS` it is declared dead (`RuntimeError`), which makes
  `run()` brake both tracks.

### Safety / failure paths

- **Motor watchdog**: the Motoron's own command timeout is left enabled on
  purpose. If the control loop ever hangs, the motor controller stops both
  tracks on its own within ~1.5 s, independent of the firmware.
- **Start signal drop**: as soon as `start_requested()` goes false,
  `stop_all()` brakes both tracks hard -- safe to call at any time, including
  as part of normal "waiting for start" idling.
- **Loop exceptions**: `run()` wraps its main loop in a `try/except` that
  calls `stop_all()` before re-raising, so the vehicle brakes rather than
  coasting (or continuing to drive) if something in the loop throws.

## Tuning knobs

Every bench-tunable constant -- distance clamp, speed, steering gain,
smoothing window -- lives in `config.py`, each with a comment explaining what
it controls. The wiring-related constants are covered in the pin table
above; the rest are covered inline above where each one affects the control
law.

## File map

Project-authored code:

| File | Role |
|---|---|
| `boot.py` | MicroPython boot hook; currently an unmodified stub |
| `config.py` | All tunable constants: pins, speeds, distance clamp, steering gain |
| `main.py` | Entry point: hardware setup, `compute()` control law, and the `run()` main loop |
| `lib/lidar_sensor.py` | Brings up the I2C bus, the VL53L0X middle sensor and the two VL53L4CX side sensors, exposes smoothed `las_avstand()` |

Vendored third-party drivers (already documented upstream -- used as-is, not
re-documented here):

| File | Source | Used for |
|---|---|---|
| `lib/motoron.py` | Pololu Motoron Python library | I2C/serial motor controller driver (`MotoronI2C`) |
| `lib/motoron_protocol.py` | Pololu Motoron Python library | Protocol constants used internally by `motoron.py` |
| `lib/vl53l0x.py` | Adafruit CircuitPython VL53L0X driver (MicroPython port) | Middle time-of-flight sensor driver |
| `lib/vl53l4cx.py` | Copied from `..\optimization\lib\` | Left/right time-of-flight sensor driver |

## Relationship to `optimization\`

This project is a sibling of `..\optimization\`, the Ackermann-steered RC car
firmware, adapted for a different physical chassis. The sensor stack
(`lib/lidar_sensor.py`, `lib/vl53l0x.py`, `lib/vl53l4cx.py`) is copied over
with only the XSHUT pins changed, and the Motoron driver
(`lib/motoron.py`, `lib/motoron_protocol.py`) is copied unchanged; `config.py`
and `main.py` are rewritten to drop the steering servo and drive two
independent track channels instead of one drive motor plus a servo.
