import sys
sys.path.append('/home/apeters/project/env/src/Functions')
import control_motors_and_servo
import keyboard
while True:
    try:
        if keyboard.is_pressed('w'):
            control_motors_and_servo(1, 0)  # Move forward
        elif keyboard.is_pressed('a'):
            print("Left")
        elif keyboard.is_pressed('s'):
            print("Backward")
        elif keyboard.is_pressed('d'):
            print("Right")
        
    except KeyboardInterrupt:
        GPIO.cleanup()