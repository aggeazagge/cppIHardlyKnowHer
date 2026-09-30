import motoron
from machine import Pin

# Pin definitions
START_MODULE = 17
I2C_SDA_PIN = 11
I2C_SCL_PIN = 12

# Motor bus setttings
I2C_BUS_ID = 0

# Motor acceleration/deceleration
MOTOR_MAX_ACCEL = 300
MOTOR_MAX_DECEL = 140 # 140 max utan att skickade tillbaka för mycket

# Motor speed (Motoron units, 0..800) -- both tracks run at this speed all
# the time; steering comes from slowing the inside track, not from throttling
# the whole vehicle down (see TURN_SLOWDOWN below).
CRUISE_SPEED = 250
MOTOR_MAX_SPEED = 800

# Braking (Motoron units, 0..800 -- separate scale from speed above: 0 = coast,
# 800 = full brake). Used by stop_all() when the start signal drops.
MOTOR_BRAKE_AMOUNT = 800

# Motoron drive motors -- one channel per track
MOTORON_ADDRESS = 16     # 7-bit I2C address (Motoron default)
MOTOR_CHANNEL_LEFT = 1
MOTOR_CHANNEL_RIGHT = 2

# Start module (autonomous-RC start device on START_MODULE / GPIO2)
START_ACTIVE_LOW = False  # car runs when the pin reads this level; flip to run on HIGH
START_PIN_PULL = "down"    # "up" | "down" | None -- internal pull resistor on GPIO2

# Obstacle-distance clamp (mm) -- readings >= this (or invalid) are treated as
# "open" by lib/lidar_sensor.py, since the VL53L0X is not trusted past ~1.2 m.
DIST_MAX = 1200

# Control tuning
STEER_SPAN_MM = 500.0     # (dr - dl) difference that maps to full steering lock
TURN_SLOWDOWN = 0.6       # inside-track speed *= (1 - TURN_SLOWDOWN * |steer fraction|)
LOOP_DELAY_MS = 10        # the blocking sensor reads already pace the loop

# VL53L0X measurement timing budget (microseconds). The driver enforces a
# hard floor of 20000 (see measurement_timing_budget setter in vl53l0x.py);
# using that floor minimizes per-measurement latency at the cost of some
# range/precision headroom this ~1 m track doesn't need. Combined with
# continuous ranging mode (lib/lidar_sensor.py), this is what makes
# las_avstand() fast.
SENSOR_TIMING_BUDGET_US = 33000
# DO NOT lower this or the signal rate limit (driver default 0.25 MCPS)
# without the read timeout below in place: aggressive tuning (e.g. signal
# rate 0.1) made a sensor stop reporting data and the car froze.

# VL53L4CX (left/right sensors) timing budget in milliseconds, 10..200 (see
# set_timing() in lib/vl53l4cx.py). Matches the middle sensor's ~33 ms so all
# three deliver new data at about the same rate.
SIDE_SENSOR_TIMING_BUDGET_MS = 33

# Rolling-median window (samples) for each distance sensor. A single
# spiky/erroneous reading is dropped entirely instead of dragging the value
# the way a mean would. Keep it odd (3 or 5); 1 disables filtering.
SENSOR_AVG_SAMPLES = 3

# Sensor failure handling (see _Channel in lib/lidar_sensor.py). A read that
# gets no new data within SENSOR_READ_TIMEOUT_MS (~3x the timing budget)
# holds the last value instead of hanging. After SENSOR_RESTART_AFTER_FAILS
# consecutive failures the sensor's continuous mode is restarted; after
# SENSOR_MAX_FAILS (~1 s) it's declared dead and the tracks brake.
SENSOR_READ_TIMEOUT_MS = 100
SENSOR_RESTART_AFTER_FAILS = 3
SENSOR_MAX_FAILS = 10
