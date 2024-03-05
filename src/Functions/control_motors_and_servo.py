import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')

from robot_hat import Servo
from robot_hat import Motors
import time


# Create Motor object
motors = Motors()

# Create Servo object with PWM object

servo_CameraYaw = Servo("P0")
servo_CameraPitch = Servo("P1")
servo_Steer = Servo("P2")

# Setup left and right motors (Drivers POV)?? i dont think this does anything tbh
motors.set_left_id(1)
motors.set_right_id(2)


def control_motors_and_servo(longitudinal_speed, steering_angle):
    # Code to control motor speed and direction
        # Motor 1 clockwise at 100% speed
    motors[1].speed(longitudinal_speed) #Left Wheel
    # Motor 2 clockwise at 100% speed
    motors[2].speed(-longitudinal_speed) #Right Wheel
    servo_Steer(steering_angle)
    pass
