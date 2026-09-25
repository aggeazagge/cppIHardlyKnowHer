from lib.servo import Servo

steering = Servo(pin_id=18, min_deg=-70.0, max_deg=180.0)  # Fix 4: Full range init

def set_steering_angle(angle):
    # Safe mechanical limits for RC car steering linkages
    safe_angle = max(60, min(120, angle))
    steering.write(safe_angle)
     