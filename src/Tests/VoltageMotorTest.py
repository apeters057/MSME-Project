import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')

import time
from robot_hat import Servo ,Motors
from VoltageRead import VoltageRead_ADS1115
# Create Motor and Servo object
motors = Motors()
motors.set_left_id(1)
motors.set_right_id(2)
# motors.set_voltage_id(3

while True:
    
    #Set up Motor Direction Speed
    # Motor 1 clockwise at 100% speed
    motors[1].speed(0) #Left
    
    #motors[2].speed(0) #Left
    motors.stop()
    #formated_voltage = "Motor Voltage: {:.4f} V".format(VoltageRead_ADS1115)
    #print(formated_voltage())
    print(VoltageRead_ADS1115())
    time.sleep(1)
                                                                                                                                         