# Huggorm 1.0 – autonomous tracked chassis firmware

MicroPython firmware for a tracked, skid-steer chassis with two independent
tracks and no steering servo. It drives itself down a walled track about 1 m
wide, using three time-of-flight sensors to stay centered: VL53L4CX sensors on
the left and right, angled outward, and a VL53L0X in the middle. Both tracks run
at `CRUISE_SPEED`, and the chassis steers by slowing the inside track. There is
no reverse.

## Hardware / wiring

ESP32-family MicroPython board. The pin numbers come from `config.py` and
`lib/lidar_sensor.py`.

| Signal | Pin | Notes |
|---|---|---|
| I2C SDA / SCL | GPIO11 / GPIO12 | shared bus for the sensors and the Motoron, 400 kHz |
| Left ToF (VL53L4CX) XSHUT | GPIO8 | reassigned to `0x30` |
| Middle ToF (VL53L0X) XSHUT | GPIO6 | default `0x29` |
| Right ToF (VL53L4CX) XSHUT | GPIO7 | reassigned to `0x31` |
| Start module | GPIO17 | level and pull set in `config.py` |

The Motoron is at `0x10` and drives the left track on channel 1 and the right
track on channel 2.

All three sensors power up at address `0x29`. To fix that, `lidar_sensor.py`
holds them in reset with XSHUT and then releases them one at a time, giving each
its own address. This has to happen before the Motoron is set up, because
`main.py` reuses the same I2C bus.

## How it runs

`boot.py` is the stock stub. `main.py` sets up the Motoron and calls `run()`,
an infinite loop that only drives while the start pin is active. While the pin
is inactive, it brakes both tracks.

To deploy, copy `boot.py`, `config.py`, `main.py` and `lib/` to the board and
power-cycle it.

## Control loop

Every ~10 ms (`LOOP_DELAY_MS`): read sensors → `compute(dl, dr)` → set track
speeds.

- **Steering fraction:** `dr - dl` is clamped to `±STEER_SPAN_MM` and turned
  into a fraction from -1 (full left) to +1 (full right).
- **Differential speed:** only the inside track slows down:
  `left = CRUISE_SPEED * (1 - TURN_SLOWDOWN * max(0, -frac))`, and the same for
  `right` with `max(0, frac)`. The outside track always runs at full speed.
- The middle sensor is read and logged but not used for control.
- There is no reverse. A pivot turn would be possible on this chassis and could
  be added if `TURN_SLOWDOWN` isn't enough for sharp corners.

## Sensors and safety

- Readings of `0` or `>= DIST_MAX` count as "open", invalid range statuses
  are ignored, and a rolling median (`SENSOR_AVG_SAMPLES`, keep it odd) removes
  spikes.
- The sensors run in continuous mode (about 33 ms timing budget), so reading
  them is quick. A read that times out keeps the last value. A sensor is
  restarted after `SENSOR_RESTART_AFTER_FAILS` failures and declared dead after
  `SENSOR_MAX_FAILS`, which brakes the chassis.
- The Motoron command timeout is left on, so the tracks stop within about
  1.5 s if the loop hangs.
- If the start signal drops or the loop throws, `stop_all()` brakes both
  tracks.

## Tuning

Every bench-tunable constant is in `config.py` with a comment.

## File map

| File | Role |
|---|---|
| `boot.py` | MicroPython boot hook (unmodified stub) |
| `config.py` | All pins and tunable constants |
| `main.py` | Hardware setup, `compute()` and the `run()` loop |
| `lib/lidar_sensor.py` | I2C bus, sensor bring-up and smoothed `las_avstand()` |
| `lib/motoron.py`, `lib/motoron_protocol.py` | Pololu Motoron driver (vendored) |
| `lib/vl53l0x.py`, `lib/vl53l4cx.py` | ToF sensor drivers (vendored) |
