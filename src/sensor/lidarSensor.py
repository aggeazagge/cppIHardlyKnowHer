import machine
import time
import lib.lidarSensor.vl53l0x as vl53l0x


# I2C
i2c = machine.I2C(
    0,
    sda=machine.Pin(11),
    scl=machine.Pin(12),
    freq=400000
)


# XSHUT-pin
xshut_left = machine.Pin(8, machine.Pin.OUT)
xshut_middle = machine.Pin(6, machine.Pin.OUT)
xshut_right = machine.Pin(7, machine.Pin.OUT)

# turn off all sensor
xshut_left.value(0)
xshut_middle.value(0)
xshut_right.value(0)
time.sleep(0.1)


# Start left-sensor
xshut_left.value(1)
time.sleep(0.1)

left = vl53l0x.VL53L0X(i2c)
left.set_address(0x30)


# Start right-sensor
xshut_right.value(1)
time.sleep(0.1)

right = vl53l0x.VL53L0X(i2c)
right.set_address(0x31)

# Start middle-sensor
xshut_middle.value(1)
time.sleep(0.1)

middle = vl53l0x.VL53L0X(i2c)


# Funktion för att läsa alla sensorer
def distance():
    distance_left = left.range
    distance_middle = middle.range
    distance_right = right.range

    return distance_left, distance_middle, distance_right