# Autonomous car firmware

MicroPython firmware for a small autonomous RC car. The car drives itself down a
walled track roughly 1 m wide, using three time-of-flight distance sensors to
stay centered between the walls and slow down for obstacles ahead. There is no
reverse gear: dead ends and sharp corners are handled by steering harder into
the turn rather than backing out.

This README exists to help you (re-)learn how the project fits together —
it explains not just *what* each piece does but *why* it's built that way.

## Hardware / wiring

The board is an ESP32-family MicroPython target (see `.vscode/settings.json`,
which points the ESP-IDF extension at the toolchain). All pin numbers below
come from `config.py` and `lib/lidar_sensor.py`.

| Signal | Pin | Notes |
|---|---|---|
| I2C SDA | GPIO11 | shared bus — sensors + Motoron |
| I2C SCL | GPIO12 | 400 kHz |
| Left ToF sensor XSHUT | GPIO8 | reassigned to I2C address `0x30` |
| Middle ToF sensor XSHUT | GPIO6 | left at the VL53L0X power-on default, `0x29` |
| Right ToF sensor XSHUT | GPIO7 | reassigned to I2C address `0x31` |
| Steering servo signal | GPIO18 | 50 Hz PWM, 544–2400 µs pulse width |
| Start-module input | GPIO17 | active level / pull configurable, see below |

The Motoron motor controller sits on the same I2C bus at address `0x10`
(`config.MOTORON_ADDRESS = 16`), driving the single drive motor on channel 1.

**Why the sensors need a startup sequence:** all three VL53L0X sensors power
up at the same default I2C address. `lib/lidar_sensor.py` handles this by
holding all three in reset via their XSHUT pins, then releasing them one at a
time and calling `set_address()` before releasing the next — that's how the
left, middle, and right sensors end up addressable independently on one bus.
This has to happen *before* the Motoron is initialized in `main.py`, since
`main.py` reuses the I2C bus object that `lidar_sensor.py` already created.

Steering angles (`STEER_CENTER_DEG` / `STEER_LEFT_DEG` / `STEER_RIGHT_DEG`) and
the start-pin active level/pull (`START_ACTIVE_LOW`, `START_PIN_PULL`) are
bench-calibration values in `config.py` — they depend on how the specific
servo horn and start module are physically mounted.

## How it runs

MicroPython has a fixed boot convention: on every power-up (or wake from
deep sleep), it runs `boot.py` first, then `main.py`. In this project:

- **`boot.py`** is currently just the stock stub MicroPython ships with
  (commented-out debug/WebREPL setup) — it does no project-specific work.
- **`main.py`** is the real entry point. Importing it has side effects: it
  sets up the Motoron controller and steering servo, and calls `run()`
  unconditionally at the bottom of the file. `run()` is an infinite loop —
  it *is* the program, not just a function you call from somewhere else.

The loop is gated by the start pin: it only drives when `start_requested()`
reads the configured "go" level; otherwise it brakes, re-centers the wheels,
and waits.

> Note: nothing in this repo documents the flashing/deployment step itself
> (no `requirements.txt`, no `mpremote`/`ampy` scripts). In practice, getting
> this running means copying `boot.py`, `config.py`, `main.py`, and `lib/`
> onto the device's filesystem with a MicroPython tool and power-cycling the
> board — but treat that as an inferred workflow, not something confirmed by
> the code here.

## Architecture: the control loop

The whole "brain" of the car is the `run()` loop in `main.py`, which every
~10 ms (`LOOP_DELAY_MS`) does: read sensors → compute steering & speed →
apply them → repeat.

### Why three sensors, and why they're used differently

The left and right sensors are mounted **splayed outward** (angled away from
straight-ahead), not straight forward. On a track only ~1 m wide, that means
they're always going to see a nearby wall — they're not "obstacle" sensors,
they're a **wall-centering signal**: if the right sensor sees more open space
than the left, the car is closer to the left wall than the right, and should
steer right. Only the **middle** sensor, pointed straight ahead, is used to
decide *how fast* to go.

### `compute(dl, dm, dr)` — the control law

Given left/middle/right distances in mm, this function (in `main.py`) returns
a steering angle and a speed:

1. **Steering** starts from `dr - dl`: positive means more room on the right,
   so steer right. This difference is clamped to `±STEER_SPAN_MM` and turned
   into a fraction from -1 (full left) to +1 (full right), which is then
   scaled onto the actual servo angle range around `STEER_CENTER_DEG`.
2. **Corner commitment**: if the middle sensor sees a wall closer than
   `DIST_SLOW`, that's read as "sharp corner ahead," and the steering
   fraction is multiplied by `CORNER_STEER_GAIN` (3.5×) before being
   clamped back to ±1. This is what lets the car commit hard into a turn
   instead of just gently correcting — without it, the car would clip
   corners because the wall-centering signal alone reacts too gently.
3. **Speed** comes only from the middle sensor, as a smooth ramp rather than
   a step function: full `CRUISE_SPEED` once `dm >= DIST_CLEAR`,
   `TURN_CRAWL_SPEED` once `dm <= DIST_STOP`, and linearly interpolated
   between the two in between. A smooth ramp avoids an abrupt speed change
   right at the threshold, which would otherwise show up as a jerky throttle
   change every time the middle reading crosses the line.
4. **Turn slowdown**: when there's a wall coming up (`dm < DIST_SLOW`),
   speed is further reduced in proportion to how hard the car is steering
   (`TURN_SLOWDOWN * |frac|`). This is deliberately *not* applied on a clear
   straight — a big steering fraction there is often just gentle
   wall-centering on a bend, not a sign to slow down.
5. Speed is floored at `TURN_CRAWL_SPEED` — the car never comes to a full
   stop while actively driving, since it has no reverse and needs
   forward motion to actually complete a turn.

### Why there's no reverse

If the car ends up nose-to-wall, it can't back out — it can only keep
"steering into" the situation. That's why `TURN_CRAWL_SPEED` and the
corner-commitment constants (`DIST_SLOW`, `DIST_STOP`, `CORNER_STEER_GAIN`)
have to be tuned *together*: crawl speed has to stay low enough, and the
turn tight enough, that the car's turning radius actually clears the wall
instead of stalling into it.

### The "hill blackout" heuristic

A ToF sensor measures the nearest thing directly in its beam. If the car
pitches nose-up going up an incline, all three beams can angle up and over
the walls, reading "nothing in range" — which the code would otherwise treat
as `DIST_MAX` ("wide open, go fast"), even though the walls are still right
there. The loop guards against this:

- Every iteration where **all three** sensors read `>= DIST_MAX` increments a
  `blackout_streak` counter; any other reading resets it to zero.
- Only once the streak reaches `BLACKOUT_CONFIRM_FRAMES` (5 frames) does the
  loop treat it as a real blackout, rather than one noisy frame or an
  actually-open straightaway.
- While in a confirmed blackout, the car **holds its last steering angle**
  and drops to the slower `HILL_BLIND_SPEED`, rather than trusting the
  false "clear" reading and cruising blind at full speed.

### Sensor smoothing

`lib/lidar_sensor.py` exposes `las_avstand()` ("read distance"), which reads
all three sensors and returns smoothed values:

- `_clamp_reading()` treats any raw reading of `0` or `>= DIST_MAX` as "open"
  (`DIST_MAX`) *before* it's averaged, so one garbage raw sample can't drag
  the average toward "wall right ahead."
- A small rolling average (`_RollingAvg`, window size `SENSOR_AVG_SAMPLES`)
  smooths out single spiky readings so the car doesn't jerk the wheel based
  on one bad sample — at the cost of a few frames of lag before a genuine
  corner registers. That's a deliberate trade documented in `config.py`: if
  the car starts clipping corners, the window is the first thing to shrink.
- Each sensor runs in **continuous ranging mode** with the minimum allowed
  measurement timing budget (`SENSOR_TIMING_BUDGET_US = 20000`, the floor
  enforced by `lib/vl53l0x.py`). That trades away some range/precision
  headroom the car doesn't need on a 1 m track, in exchange for making each
  `las_avstand()` call a cheap register read instead of triggering a whole
  new measurement — which is what keeps the ~10 ms loop fast enough.

### Safety / failure paths

- **Motor watchdog**: the Motoron's own command timeout is left enabled on
  purpose. If the control loop ever hangs, the motor controller stops the
  motor on its own within ~1.5 s, independent of the firmware.
- **Start signal drop**: as soon as `start_requested()` goes false,
  `stop_all()` brakes hard and re-centers the steering — safe to call at any
  time, including as part of normal "waiting for start" idling.
- **Loop exceptions**: `run()` wraps its main loop in a `try/except` that
  calls `stop_all()` before re-raising, so the car brakes rather than
  coasting (or continuing to drive) if something in the loop throws.

## Tuning knobs

Every bench-tunable constant — distance thresholds, speeds, steering
angles/gains, smoothing window, blackout parameters — lives in `config.py`,
each with a comment explaining what it controls and (where relevant) what
to try if the car's behavior needs adjusting. The wiring-related constants
are covered in the pin table above; the rest are covered inline above where
each one affects the control law.

## File map

Project-authored code:

| File | Role |
|---|---|
| `boot.py` | MicroPython boot hook; currently an unmodified stub |
| `config.py` | All tunable constants: pins, servo calibration, speeds, distance thresholds, control gains |
| `main.py` | Entry point: hardware setup, `compute()` control law, and the `run()` main loop |
| `lib/lidar_sensor.py` | Brings up the I2C bus and three VL53L0X sensors, exposes smoothed `las_avstand()` |
| `lib/servo.py` | Minimal PWM servo driver (degrees ↔ pulse width ↔ duty cycle) |

Vendored third-party drivers (already documented upstream — used as-is, not
re-documented here):

| File | Source | Used for |
|---|---|---|
| `lib/motoron.py` | Pololu Motoron Python library | I2C/serial motor controller driver (`MotoronI2C`) |
| `lib/motoron_protocol.py` | Pololu Motoron Python library | Protocol constants used internally by `motoron.py` |
| `lib/vl53l0x.py` | Adafruit CircuitPython VL53L0X driver (MicroPython port) | Time-of-flight distance sensor driver |
