# Snok 1.2 – autonomous RC car firmware

MicroPython firmware for an Ackermann-steered RC car (one drive motor and a
steering servo) that drives itself down a walled track about 1 m wide. It uses
three time-of-flight sensors to stay centered: VL53L4CX sensors on the left and
right, angled outward, and a VL53L0X in the middle. New in 1.2: the car notices
when it is stuck against an obstacle and reverses out.

## Hardware / wiring

ESP32-family MicroPython board. The pin numbers come from `config.py` and
`lib/lidar_sensor.py`.

| Signal | Pin | Notes |
|---|---|---|
| I2C SDA / SCL | GPIO11 / GPIO12 | shared bus for the sensors and the Motoron, 400 kHz |
| Left ToF (VL53L4CX) XSHUT | GPIO7 | reassigned to `0x30` |
| Middle ToF (VL53L0X) XSHUT | GPIO6 | default `0x29` |
| Right ToF (VL53L4CX) XSHUT | GPIO8 | reassigned to `0x31` |
| Steering servo | GPIO18 | 50 Hz, 544–2400 µs |
| Start module | GPIO17 | level and pull set in `config.py` |

The Motoron motor controller is at I2C address `0x10` and drives the motor on
channel 1. `MOTOR_INVERT = True` because the motor wires are swapped.

All three sensors power up at address `0x29`. To fix that, `lidar_sensor.py`
holds them in reset with XSHUT and then releases them one at a time, giving each
its own address. This runs when the module is imported, before the Motoron is
set up.

## How it runs

`main.py` is the entry point and calls `run()` at the bottom. `run()` is an
infinite loop that only drives while the start pin is active. While the pin is
inactive, it brakes and centers the wheels with `stop_all()`.

To deploy, copy `config.py`, `main.py` and `lib/` to the board (for example with
mpremote or Thonny) and power-cycle it.

## Control loop

Each iteration (`LOOP_DELAY_MS`) does: read sensors → compute steering and
speed → apply → repeat.

- **Steering:** `dr - dl` is clamped to `±STEER_SPAN_MM` and turned into a
  fraction from -1 (full left) to +1 (full right), which is then mapped to a
  servo angle around `STEER_CENTER_DEG`. The side sensors keep the car centered
  between the walls. They are not used as obstacle sensors.
- **Corners:** when the middle sensor reads below `DIST_SLOW`, the steering
  fraction is multiplied by `CORNER_STEER_GAIN` so the car commits harder to the
  turn, and speed drops by `TURN_SLOWDOWN`.
- **Speed:** speed depends only on the middle sensor. The car drives at
  `CRUISE_SPEED` at `DIST_CLEAR` or more, ramps down toward `DIST_STOP`, and
  never goes below `TURN_CRAWL_SPEED`.
- **Stuck detection (new):** `check_stuck()` reports the car as stuck when an
  obstacle is less than 250 mm ahead and the distance has changed by less than
  15 mm for 2 s. `backWard()` then reverses at `BACK_SPEED` for 1 s with the
  wheels straight.
- **Hill blackout:** if all three sensors read `DIST_MAX` for
  `BLACKOUT_CONFIRM_FRAMES` frames in a row, the car is probably pitched up on a
  hill and looking over the walls. It then holds half of the last steering
  angle and drives at `HILL_BLIND_SPEED`.

## Sensors and safety

- Readings of `0` or `>= DIST_MAX` count as "open", invalid range statuses
  are ignored, and a rolling median (`SENSOR_AVG_SAMPLES`) removes spikes.
- The sensors run in continuous mode, so reading them is quick. A read that
  times out keeps the last value. A sensor is restarted after
  `SENSOR_RESTART_AFTER_FAILS` failures and declared dead after
  `SENSOR_MAX_FAILS`, which brakes the car.
- The Motoron command timeout is left on, so the motor stops within about
  1.5 s if the loop hangs.
- Any exception in `run()` calls `stop_all()` before re-raising it.

More detail on the sensor stack is in [huggorm_1.0/README.md](../huggorm_1.0/README.md).

## Tuning

Every bench-tunable constant is in `config.py` with a comment: speeds,
distance thresholds, steering angles and gains, and sensor timing. Calibrate
`STEER_CENTER_DEG`, `STEER_LEFT_DEG` and `STEER_RIGHT_DEG` on the bench.

## File map

| File | Role |
|---|---|
| `config.py` | All pins and tunable constants |
| `main.py` | Hardware setup, `compute()`, stuck detection and reverse, the `run()` loop |
| `lib/lidar_sensor.py` | I2C bus, sensor bring-up and smoothed `las_avstand()` |
| `lib/servo.py` | Steering servo driver |
| `lib/motoron.py`, `lib/motoron_protocol.py` | Pololu Motoron driver (vendored) |
| `lib/vl53l0x.py`, `lib/vl53l4cx.py` | ToF sensor drivers (vendored) |
