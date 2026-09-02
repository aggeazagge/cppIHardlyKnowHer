import time
from src.sensor.lidarSensor import distance
from src.motor import motor
motor.init()

car_running = False
last_button_state = motor.button_state()


while True:
    current_button_state = motor.button_state()
    if last_button_state == 1 and current_button_state == 0:

        car_running = not car_running

        if car_running:
            print("START")
        else:
            print("STOP")
            motor.stop()

        time.sleep_ms(200)

    last_button_state = current_button_state

    # Kör bilen
    if car_running:
        motor.forward()

    time.sleep_ms(10)

    left, middle, right = distance()

    print("Left:", left, "mm")
    print("Middle:", middle, "mm")
    print("Right:", right, "mm")

    time.sleep_ms(50)