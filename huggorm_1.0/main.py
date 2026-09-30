# Autonomous driving loop for the tracked (skid-steer) chassis.
#
# MicroPython runs boot.py then main.py on power-up, so this file is the entry
# point. It wires together the two libraries in this project:
#   - lidar_sensor : three ToF distance sensors -- VL53L4CX left/right,
#                    VL53L0X middle
#   - motoron      : Pololu Motoron controller for the two tracks
# and runs a reactive obstacle-avoidance loop: both tracks always run at
# CRUISE_SPEED, and steering is done by slowing the inside track for a turn
# (e.g. the left track slows for a left turn), never by throttling the whole
# vehicle down. There is no reverse -- dead ends are handled by committing
# harder to the turn (see compute()) rather than backing out.

import time
from machine import Pin

import config
import lidar_sensor          # importing this brings up the I2C bus + 3 sensors
import motoron


# --- Hardware setup ---------------------------------------------------------

# Reuse the I2C bus that lidar_sensor already created on the GPIO11/12 pins;
# the Motoron (0x10) does not collide with the VL53L0X addresses.
mc = motoron.MotoronI2C(bus=lidar_sensor.i2c, address=config.MOTORON_ADDRESS)
mc.reinitialize()
mc.clear_reset_flag()
for channel in (config.MOTOR_CHANNEL_LEFT, config.MOTOR_CHANNEL_RIGHT):
    mc.set_max_acceleration(channel, config.MOTOR_MAX_ACCEL)
    mc.set_max_deceleration(channel, config.MOTOR_MAX_DECEL)
# The Motoron command timeout stays enabled on purpose: if this loop ever
# stalls, both tracks stop on their own within ~1.5 s.

_PULL = {"up": Pin.PULL_UP, "down": Pin.PULL_DOWN, None: None}[config.START_PIN_PULL]
start_pin = Pin(config.START_MODULE, Pin.IN, _PULL)


# --- Helpers ---------------------------------------------------------------

def clamp(value, low, high):
    return low if value < low else high if value > high else value


def start_requested():
    """True when the start module says 'go'."""
    active_level = 0 if config.START_ACTIVE_LOW else 1
    return start_pin.value() == active_level


def drive(left_speed, right_speed):
    left_speed = int(clamp(left_speed, -config.MOTOR_MAX_SPEED, config.MOTOR_MAX_SPEED))
    right_speed = int(clamp(right_speed, -config.MOTOR_MAX_SPEED, config.MOTOR_MAX_SPEED))
    mc.set_speed(config.MOTOR_CHANNEL_LEFT, left_speed)
    mc.set_speed(config.MOTOR_CHANNEL_RIGHT, right_speed)


def stop_all():
    """Brake both tracks hard. Safe to call any time."""
    mc.set_braking_now(config.MOTOR_CHANNEL_LEFT, config.MOTOR_BRAKE_AMOUNT)
    mc.set_braking_now(config.MOTOR_CHANNEL_RIGHT, config.MOTOR_BRAKE_AMOUNT)


# --- Control law ---------------------------------------------------------

def compute(dl, dr):
    """Map left/right wall distances in mm to (left_speed, right_speed).

    The sensors are splayed (left looks forward-left, right forward-right), so on
    a ~1 m track the side readings are a wall-centering signal, not obstacles:
    steer to keep dl ~= dr. Both tracks default to CRUISE_SPEED -- this chassis
    always drives at full speed -- and only the inside track for the current
    turn direction is slowed, in proportion to how hard the turn is.
    """
    # frac: -1 = hard left, +1 = hard right (same sign convention as dr - dl).
    diff = clamp(dr - dl, -config.STEER_SPAN_MM, config.STEER_SPAN_MM)
    frac = diff / config.STEER_SPAN_MM

    left_speed = config.CRUISE_SPEED * (1.0 - config.TURN_SLOWDOWN * max(0.0, -frac))
    right_speed = config.CRUISE_SPEED * (1.0 - config.TURN_SLOWDOWN * max(0.0, frac))

    return left_speed, right_speed


# --- Main loop ---------------------------------------------------------

def run():
    print("autonomous loop ready -- waiting for start signal")
    was_running = False
    try:
        while True:
            if not start_requested():
                if was_running:
                    print("start signal off -- stopping")
                stop_all()
                was_running = False
                time.sleep_ms(config.LOOP_DELAY_MS)
                continue

            if not was_running:
                print("start signal on -- driving")
                was_running = True

            dl, dm, dr = lidar_sensor.las_avstand()

            left_speed, right_speed = compute(dl, dr)
            drive(left_speed, right_speed)
            print(dl, dm, dr, "->", int(left_speed), int(right_speed))

            time.sleep_ms(config.LOOP_DELAY_MS)
    except (KeyboardInterrupt, Exception) as e:
        stop_all()
        print("stopped:", e)
        raise


run()
