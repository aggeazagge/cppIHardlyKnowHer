# Autonomous driving loop.
#
# MicroPython runs boot.py then main.py on power-up, so this file is the entry
# point. It wires together the three libraries in this project:
#   - lidar_sensor : three distance sensors (VL53L4CX left/right, VL53L0X middle)
#   - motoron      : Pololu Motoron controller for the drive motor
#   - servo        : the steering servo
# and runs a reactive obstacle-avoidance loop: steer toward open space and slow
# down as obstacles get closer. There is no reverse -- dead ends are handled by
# committing harder to the turn (see compute()) rather than backing out.

import time
from machine import Pin

import config
import lidar_sensor          # importing this brings up the I2C bus + 3 sensors
import motoron
from servo import Servo


# --- Hardware setup ---------------------------------------------------------

# Reuse the I2C bus that lidar_sensor already created on the GPIO11/12 pins;
# the Motoron (0x10) does not collide with the VL53L0X addresses.
mc = motoron.MotoronI2C(bus=lidar_sensor.i2c, address=config.MOTORON_ADDRESS)
mc.reinitialize()
mc.clear_reset_flag()
mc.set_max_acceleration(config.MOTOR_CHANNEL, config.MOTOR_MAX_ACCEL)
mc.set_max_deceleration(config.MOTOR_CHANNEL, config.MOTOR_MAX_DECEL)
# The Motoron command timeout stays enabled on purpose: if this loop ever
# stalls, the motor stops on its own within ~1.5 s.

steering = Servo(
    config.SERVO_PIN,
    min_us=config.SERVO_MIN_US,
    max_us=config.SERVO_MAX_US,
    min_deg=config.SERVO_MIN_DEG,
    max_deg=config.SERVO_MAX_DEG,
    freq=config.SERVO_FREQ_HZ,
)

_PULL = {"up": Pin.PULL_UP, "down": Pin.PULL_DOWN, None: None}[config.START_PIN_PULL]
start_pin = Pin(config.START_MODULE, Pin.IN, _PULL)


# --- Helpers ---------------------------------------------------------------

def clamp(value, low, high):
    return low if value < low else high if value > high else value


def start_requested():
    """True when the start module says 'go'."""
    active_level = 0 if config.START_ACTIVE_LOW else 1
    return start_pin.value() == active_level


def set_steer(angle_deg):
    lo = min(config.STEER_LEFT_DEG, config.STEER_RIGHT_DEG)
    hi = max(config.STEER_LEFT_DEG, config.STEER_RIGHT_DEG)
    steering.write(clamp(angle_deg, lo, hi))


def drive(speed):
    speed = int(clamp(speed, -config.MOTOR_MAX_SPEED, config.MOTOR_MAX_SPEED))
    if config.MOTOR_INVERT:
        speed = -speed
    mc.set_speed(config.MOTOR_CHANNEL, speed)


def stop_all():
    """Brake hard and re-center the wheels. Safe to call any time."""
    mc.set_braking_now(config.MOTOR_CHANNEL, config.MOTOR_BRAKE_AMOUNT)
    set_steer(config.STEER_CENTER_DEG)


# --- Control law ---------------------------------------------------------

def compute(dl, dm, dr):
    """Map (left, middle, right) distances in mm to (steer_deg, speed).

    The sensors are splayed (left looks forward-left, right forward-right), so on
    a ~1 m track the side readings are a wall-centering signal, not obstacles:
    steer to keep dl ~= dr. Speed comes from the middle sensor alone.
    """
    # Steer to re-center between the walls. frac: -1 = hard left, +1 = hard right.
    diff = clamp(dr - dl, -config.STEER_SPAN_MM, config.STEER_SPAN_MM)
    frac = diff / config.STEER_SPAN_MM

    # Wall dead ahead (sharp corner) -> commit harder to the turn.
    if dm < config.DIST_SLOW:
        frac = clamp(frac * config.CORNER_STEER_GAIN, -1.0, 1.0)

    half = min(
        config.STEER_CENTER_DEG - config.STEER_LEFT_DEG,
        config.STEER_RIGHT_DEG - config.STEER_CENTER_DEG,
    )
    steer_deg = config.STEER_CENTER_DEG + frac * half

    # Speed from forward clearance (middle sensor) only.
    if dm >= config.DIST_CLEAR:
        speed = config.CRUISE_SPEED
    elif dm <= config.DIST_STOP:
        speed = config.TURN_CRAWL_SPEED      # nose near a wall, still turning
    else:
        span = config.DIST_CLEAR - config.DIST_STOP
        t = (dm - config.DIST_STOP) / span
        speed = config.CRUISE_SPEED * (0.4 + 0.6 * t)

    # Ease off for hard steering, but only when there's a wall coming up ahead
    # (dm < DIST_SLOW). With a clear corridor a big steering input is just
    # wall-centering / a gentle bend and shouldn't cost speed.
    if dm < config.DIST_SLOW:
        speed *= (1.0 - config.TURN_SLOWDOWN * abs(frac))
    speed = max(speed, config.TURN_CRAWL_SPEED)

    return steer_deg, speed
def backWard():
    print("BACKAR!")

    # Håll hjulen raka
    set_steer(config.STEER_CENTER_DEG)

    # Backa
    drive(config.BACK_SPEED)
    time.sleep_ms(1000)

    # Stoppa
    stop_all()

    # Nollställ fastkörningshistoriken
    global stuck_start, last_middle_distance

    stuck_start = None
    last_middle_distance = None




STUCK_DISTANCE = 250       # mm - hinder närmare än 25 cm
STUCK_TIME_MS = 2000       # 2 sekunder
STUCK_CHANGE_MM = 15        # max förändring i avstånd


stuck_start = None
last_middle_distance = None


STUCK_DISTANCE = 250       # mm
STUCK_TIME_MS = 2000       # 2 sekunder
STUCK_CHANGE_MM = 15       # max förändring

stuck_start = None
last_distances = None

def check_stuck(dl, dm, dr, speed):
    global stuck_start, last_middle_distance

    # Om bilen inte kör framåt -> återställ
    if speed <= 0:
        stuck_start = None
        last_middle_distance = None
        return False

    now = time.ticks_ms()

    # Vi bryr oss bara om ett hinder nära framför bilen
    if dm > STUCK_DISTANCE:
        stuck_start = None
        last_middle_distance = dm
        return False

    # Första mätningen
    if last_middle_distance is None:
        last_middle_distance = dm
        stuck_start = now
        return False

    # Har avståndet ändrats tillräckligt?
    change = abs(dm - last_middle_distance)

    if change > STUCK_CHANGE_MM:
        # Bilen rör sig -> inte fast
        stuck_start = None
        last_middle_distance = dm
        return False

    # Avståndet ändras nästan inte
    if stuck_start is None:
        stuck_start = now

    elapsed = time.ticks_diff(now, stuck_start)

    if elapsed >= STUCK_TIME_MS:
        print("FASTNAT FRAMFÖR! Avstånd:", dm)
        stuck_start = None
        last_middle_distance = dm
        return True

    last_middle_distance = dm
    return False


# mätte spänningen på data pinnen sda
# --- Main loop ---------------------------------------------------------

def run():
    print("autonomous loop ready -- waiting for start signal")
    was_running = False
    last_steer_deg = config.STEER_CENTER_DEG
    blackout_streak = 0
    try:
        while True:
            if not start_requested():
                if was_running:
                    print("start signal off -- stopping")
                stop_all()
                was_running = False
                last_steer_deg = config.STEER_CENTER_DEG
                blackout_streak = 0
                time.sleep_ms(config.LOOP_DELAY_MS)
                continue

            if not was_running:
                print("start signal on -- driving")
                was_running = True

            dl, dm, dr = lidar_sensor.las_avstand()

            # All three sensors maxed for several frames in a row -> almost
            # certainly a hill blind spot (beams pointing over the walls),
            # not a genuinely open track. Hold the last heading and crawl
            # instead of trusting "open" and cruising at full speed.
            if dl >= config.DIST_MAX and dm >= config.DIST_MAX and dr >= config.DIST_MAX:
                blackout_streak += 1
            else:
                blackout_streak = 0

            if blackout_streak >= config.BLACKOUT_CONFIRM_FRAMES:
                steer_deg = last_steer_deg/2
                speed = config.HILL_BLIND_SPEED
                blackout_tag = " BLACKOUT"
            else:
                steer_deg, speed = compute(dl, dm, dr)
                if check_stuck(dl, dm, dr, speed):
                    print("Bilen verkar ha fastnat -> backar!")
                    backWard()
                    continue
                last_steer_deg = steer_deg
                blackout_tag = ""

            set_steer(steer_deg)
            drive(speed)
            print(dl, dm, dr, "->", round(steer_deg), int(speed), blackout_tag)

            time.sleep_ms(config.LOOP_DELAY_MS)
    except (KeyboardInterrupt, Exception) as e:
        stop_all()
        print("stopped:", e)
        raise


run()
