import machine
import time
import vl53l0x

# I2C
i2c = machine.I2C(
    0,
    sda=machine.Pin(11),
    scl=machine.Pin(12),
    freq=400000
)


# XSHUT-pinnar
xshut_vanster = machine.Pin(10, machine.Pin.OUT)
xshut_mitten = machine.Pin(8, machine.Pin.OUT)
xshut_hoger = machine.Pin(9, machine.Pin.OUT)
print(i2c.scan())

# Stäng av båda sensorerna
xshut_vanster.value(0)
xshut_mitten.value(0)
xshut_hoger.value(0)
time.sleep(0.1)

# Starta vänster sensor
xshut_vanster.value(1)
time.sleep(0.1)

vanster = vl53l0x.VL53L0X(i2c)
vanster.set_address(0x30)

# Starta höger sensor
xshut_hoger.value(1)
time.sleep(0.1)

hoger = vl53l0x.VL53L0X(i2c)
hoger.set_address(0x31)

# Starta mitten sensor
xshut_mitten.value(1)
time.sleep(0.1)

mitten = vl53l0x.VL53L0X(i2c)
#mitten.set_address(0x29)

# Mät
while True:
    avstand_vanster = vanster.range
    avstand_mitten = mitten.range
    avstand_hoger = hoger.range

    print("Vänster: {} mm | Mitten: {} mm | Höger: {} mm".format(
        avstand_vanster,
        avstand_mitten,
        avstand_hoger
    ))

    time.sleep(0.2)

    