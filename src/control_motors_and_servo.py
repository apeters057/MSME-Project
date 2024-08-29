
import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
import keyboard

from robot_hat import Servo
from robot_hat import Motors
import time
#Run through terminal --> sudo python ..
# Create Motor object
motors = Motors()
# Initial motor value
motor_value = 0  # You can set your desired initial value
# Slow down factor
slow_down_factor = 0.0001  # Adjust this value to control the speed of slowing down
steer_angle = -10 # Center


# Create Servo object with PWM object
servo_CameraYaw = Servo("P0")
servo_CameraPitch = Servo("P1")
servo_Steer = Servo("P2")

# Setup left and right motors (Drivers POV)?? i dont think this does anything tbh
motors.set_left_id(1)
motors.set_right_id(2)
#Set up Motor Direction Speed
motors[1].speed(motor_value) #Left
motors[2].speed(motor_value) #Right
##SteeringAngle## Center = -10?  ----> Limits = Right Left 30 /-30 degrees (deg)
servo_Steer.angle(steer_angle)
# Main game loop
while True:
    motors[1].speed(motor_value) #Left
    motors[2].speed(motor_value) #Right
    servo_Steer.angle(steer_angle)
    # Check for key presses
    ###Drive Motor
    if keyboard.is_pressed('up'):
        # Increase motor value (up to 100)
        motor_value = min(100, motor_value + 1)
        time.sleep(0.01)
    elif keyboard.is_pressed('down'):
        # Decrease motor value (down to 1)
        motor_value = max(1, motor_value - 1)
        time.sleep(0.01)
        print(motor_value)
    elif keyboard.is_pressed('w') :
        # Set motor value to the maximum (100)
        motor_value = 100
    elif keyboard.is_pressed('s') :
        # Slow down motor value
        motor_value *= slow_down_factor
        
    #Steer Servo
    elif keyboard.is_pressed('left'):
        # Increase motor value (up to 100)
        steer_angle = min(30, steer_angle + 1)
        time.sleep(0.01)
    elif keyboard.is_pressed('right'):
        # Increase motor value (up to 100)
        steer_angle = max(-30, steer_angle - 1)
        time.sleep(0.01)
    if keyboard.is_pressed('q'):
        # Increase motor value (up to 100)
        steer_angle = -10
        




