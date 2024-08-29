import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')

from robot_hat import Motors
import time
# Create Motor object
motors = Motors()

# Setup left and right motors (Drivers POV)?? i dont think this does anything tbh
motors.set_left_id(1)
motors.set_right_id(2)
motors.set_voltage_id(3)

#Set up Motor Direction Speed
# Motor 1 clockwise at 100% speed
motors[1].speed(50) #Left
# Motor 2 clockwise at 100% speed
motors[2].speed(50) #Right

# Stop all motors
time.sleep(3)
motors.stop()

#### If you found a motor is running in the wrong direction ####
# Use these function to correct it
#motors.set_left_reverse()
#motors.set_right_reverse()
# Go forward and see if both motor directions are correct
#motors.forward(50)
# Run forward again and see if both motor directions are correct

