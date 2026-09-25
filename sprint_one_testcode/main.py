import time
import motoron
from machine import I2C, Pin
from servo import Servo

# Fix 2: Put button and servo on separate pins (e.g. Button on D4, Servo on D2)
button = Pin(2, Pin.IN, Pin.PULL_UP)
steering = Servo(pin_id=5, min_deg=-70.0, max_deg=180.0)  # Fix 4: Full range init

bus = I2C(0, scl=Pin(12), sda=Pin(11))
mc = motoron.MotoronI2C(bus=bus)

# Reinitialize and clear reset flags
mc.reinitialize()
mc.disable_crc()
mc.clear_reset_flag()

# Configure motors
mc.set_max_acceleration(1, 140)
mc.set_max_deceleration(1, 300)
mc.set_max_acceleration(2, 200)
mc.set_max_deceleration(2, 300)
mc.set_max_acceleration(3, 80)
mc.set_max_deceleration(3, 300)

def set_steering_angle(angle):
    # Safe mechanical limits for RC car steering linkages
    safe_angle = max(60, min(120, angle))
    steering.write(safe_angle)

def stop_all_motors():
    """Safely brings all motors to a stop and centers steering."""
    mc.set_speed(1, 0)
    mc.set_speed(2, 0)
    mc.set_speed(3, 0)
    set_steering_angle(90)

# Initial hardware state
stop_all_motors()
car_running = False
last_button_state = button.value()

print("System Ready! Press button on D4 to Start / Stop.")

try:
    while True:
        current_button_state = button.value()

        # Edge detection: Unpressed (1) -> Pressed (0)
        if last_button_state == 1 and current_button_state == 0:
            car_running = not car_running
            
            if car_running:
                print(">>> STARTING MOTORS <<<")
            else:
                print(">>> PAUSING MOTORS <<<")
                stop_all_motors()
            
            time.sleep_ms(200)  # Debounce delay

        last_button_state = current_button_state

        # Drive logic (Non-blocking so button checks happen fast)
        if car_running:
            # Motor drive logic
            if time.ticks_ms() & 2048:
                mc.set_speed(1, 150)
                set_steering_angle(60)   # Turn left while moving forward
            else:
                mc.set_speed(1, 150)
                set_steering_angle(120)  # Turn right while reversing

            mc.set_speed(2, 100)
            mc.set_speed(3, -100)

        time.sleep_ms(10)  # Fast 10ms loop ensures instant button response

except KeyboardInterrupt:
    stop_all_motors()
    print("Program terminated via keyboard interrupt.")