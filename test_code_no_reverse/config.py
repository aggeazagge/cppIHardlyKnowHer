import motoron
from machine import Pin

# Pin definitions
START_MODULE = 17
SERVO_PIN = 18
I2C_SDA_PIN = 11
I2C_SCL_PIN = 12

# Servo settings
SERVO_MIN_DEG = -70.0
SERVO_MAX_DEG = 180.0

# Motor bus setttings
I2C_BUS_ID = 0

# Motor acceleration/deceleration
MOTOR_MAX_ACCEL = 300
MOTOR_MAX_DECEL = 140 # 140 max utan att skickade tillbaka för mycket

# Motor speed (Motoron units, 0..800)
CRUISE_SPEED = 45 0       # normal forward speed on a clear path
MOTOR_MAX_SPEED = 800

# Braking (Motoron units, 0..800 -- separate scale from speed above: 0 = coast,
# 800 = full brake). Used by stop_all() when the start signal drops.
MOTOR_BRAKE_AMOUNT = 800

# Motoron drive motor
MOTORON_ADDRESS = 16     # 7-bit I2C address (Motoron default)
MOTOR_CHANNEL = 1        # single drive motor on channel 1

# Start module (autonomous-RC start device on START_MODULE / GPIO2)
START_ACTIVE_LOW = False  # car runs when the pin reads this level; flip to run on HIGH
START_PIN_PULL = "down"    # "up" | "down" | None -- internal pull resistor on GPIO2

# Steering angles handed to Servo.write() -- CALIBRATE THESE ON THE BENCH.
# They must sit inside SERVO_MIN_DEG..SERVO_MAX_DEG.
STEER_CENTER_DEG = 90.0  # wheels pointing straight
STEER_LEFT_DEG = 120.0    # full left lock
STEER_RIGHT_DEG = 60.0  # full right lock

# Servo pulse-width limits (microseconds) and update rate for this servo
SERVO_MIN_US = 544.0
SERVO_MAX_US = 2400.0
SERVO_FREQ_HZ = 50

# Obstacle-distance thresholds (mm) -- sized for a ~1 m walled track. Only the
# middle sensor gates speed; the splayed side sensors always see a wall a few
# hundred mm away, so they feed steering only (see compute()).
DIST_MAX = 1200          # readings >= this (or invalid) -> "open"; the VL53L0X
                         # is not trusted past ~1.2 m
DIST_CLEAR = 650         # middle sensor >= this ahead -> full cruise
DIST_SLOW = 380          # middle sensor < this -> slow down + commit to the turn
DIST_STOP = 170          # middle sensor <= this -> crawl speed, still turning

# Control tuning
STEER_SPAN_MM = 500.0     # (dr - dl) difference that maps to full steering lock
CORNER_STEER_GAIN = 2.5   # extra steering commitment when middle < DIST_SLOW
TURN_SLOWDOWN = 0.6       # speed *= (1 - TURN_SLOWDOWN * |steer fraction|)
LOOP_DELAY_MS = 10        # the blocking sensor reads already pace the loop

# Forward crawl speed: the minimum forward speed used while turning hard.
# There is no reverse -- this must be low enough, and DIST_SLOW/DIST_STOP/
# CORNER_STEER_GAIN tight enough, that the turn radius actually clears the
# wall. Bench-tune these together if it still clips a corner.
TURN_CRAWL_SPEED = 260