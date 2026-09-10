import time
from lib.motor import motorn
from machine import I2C, Pin


# Hardware

button = Pin(2, Pin.IN, Pin.PULL_UP)
bus = I2C(0, scl=Pin(12), sda=Pin(11))
mc = motorn.MotoronI2C(bus=bus)

# Reinitialize and clear reset flags
# Motor initialization
def init():

    mc.reinitialize()
    mc.disable_crc()
    mc.clear_reset_flag()

    # Configure motors 1
    mc.set_max_acceleration(1, 140)
    mc.set_max_deceleration(1, 300)
    # Configure motors 2
    mc.set_max_acceleration(2, 200)
    mc.set_max_deceleration(2, 300)
    # Configure motors 3
    mc.set_max_acceleration(3, 80)
    mc.set_max_deceleration(3, 300)
    stop()

def stop():
    mc.set_speed(1, 0)
    mc.set_speed(2, 0)
    mc.set_speed(3, 0)


def forward():
    mc.set_speed(1, 400)
    mc.set_speed(2, 100)
    mc.set_speed(3, 100)


def backward():
    mc.set_speed(1, -150)
    mc.set_speed(2, -100)
    mc.set_speed(3, -100)


def button_state():
    return button.value()