import time
from src.sensor.lidarSensor import distance
from src.motor import motor
from src.servo.servo import set_steering_angle

motor.init()

car_running = False
last_button_state = motor.button_state()

set_steering_angle(90)

while True:

    # Läs sensorer
    left, middle, right = distance()

    # Läs knapp
    current_button_state = motor.button_state()

    # Start / stop
    if last_button_state == 0 and current_button_state == 1:

        car_running = not car_running

        if car_running:
            print("START")
            motor.forward()
            set_steering_angle(90)

        else:
            print("STOP")
            motor.stop()
            set_steering_angle(90)

        time.sleep_ms(200)

    last_button_state = current_button_state



    if car_running:

        # Something directly ahead
        if middle < 40:
            motor.stop()
            set_steering_angle(90)

            # Reverse briefly
            motor.backward()
            time.sleep_ms(2000)

            motor.stop()

        #Obstacle on the left
        elif left < 33:
            motor.backward()
            set_steering_angle(120)  # Turn right while reversing
            time.sleep_ms(2000)
            motor.forward()
            set_steering_angle(90)
            
            

        # Obstacle on the right
        elif right < 33:
            motor.backward()
            set_steering_angle(60) # Turn left while moving forward
            time.sleep_ms(2000)
            motor.forward()
            set_steering_angle(90)

        # Nothing nearby
        else:
            motor.forward()
            set_steering_angle(90)

    else:
        motor.stop()
        set_steering_angle(90)

    time.sleep_ms(50)         
        
