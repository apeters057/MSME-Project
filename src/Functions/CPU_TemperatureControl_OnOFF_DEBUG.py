import sys
# Adjust paths as needed
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
from robot_hat import pwm, Pin
import time
import os
PWM = pwm.PWM
                                                                                                                                                              
# Create a PWM object for the fan (choose the appropriate pin label)
fan_pwm = PWM('P3')  # Replace 'P3' with the correct PWM pin
#fan_pin = Pin('P3')  # Pin object for manual LOW state control if needed

# Set PWM properties
fan_pwm.freq(25050)  # Set frequency to 25 kHz, typical for fan control

# Temperature thresholds in Celsius
TEMP_THRESHOLD = 50


# Temperature to turn on the fan
TEMP_HYSTERESIS = 5  # Hysteresis to turn off after cooling down

def get_cpu_temp():
    # Read CPU temperature
    temp = os.popen("vcgencmd measure_temp").readline()
    return float(temp.replace("temp=", "").replace("'C\n", ""))

try:
    while True:
        cpu_temp = get_cpu_temp()
        print(f"Current CPU temperature: {cpu_temp}°C")  # Debugging print
        
        if cpu_temp >= TEMP_THRESHOLD:
            print("Turning fan ON")
            fan_pwm.pulse_width_percent(100)  # Full power to turn the fan on
        elif cpu_temp <= TEMP_THRESHOLD - TEMP_HYSTERESIS:
            print("Turning fan OFF")
            fan_pwm.pulse_width_percent(0)    # Attempt to turn the fan off

            # Additional check to manually set the pin LOW if PWM is insufficient
            #fan_pin.write(0)  # Ensure the fan turns off

        time.sleep(5)  # Check temperature every 5 seconds

except KeyboardInterrupt:
    print("Script stopped by user")

finally:
    print("Cleaning up: Turning fan OFF")
    fan_pwm.pulse_width_percent(0)  # Ensure the fan is off when the script stops
    #fan_pin.write(0)  # Ensure the pin is LOW for manual override
