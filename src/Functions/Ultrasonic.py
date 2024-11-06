import sys
# Adjust paths as needed
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')

import time
from robot_hat import modules, pin
Ultrasonic = modules.Ultrasonic
Pin = pin.Pin

# Define your GPIO pins for the ultrasonic sensor
TRIG_PIN = 27  #D2 #Yellow Replace with your actual GPIO pin number for trigger
ECHO_PIN = 22  #D3 # Replace with your actual GPIO pin number for echo


us = Ultrasonic(Pin("D2"), Pin("D3"))
distance = us.read()
#ultrasonic_sensor = Ultrasonic(trig=trig_pin, echo=echo_pin)

# Ultrasonic read function
def read_ultrasonic_distance():
    distance = us.read()
    return distance if distance != -1 else None
