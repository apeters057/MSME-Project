import sys
# Adjust paths as needed
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
import os
import time
import psutil

# USB hub and port configuration
HUB_ID = "1-1"  # Replace with your hub ID
PORT_ID = "3"   # Replace with your port ID

# Temperature thresholds
TEMP_ON = 59  # Temperature to turn fan on (in °C)
TEMP_OFF = 58  # Temperature to turn fan off (in °C)

def get_cpu_temp():
    # Fetch CPU temperature
    temp = psutil.sensors_temperatures()["cpu_thermal"][0].current
    return temp

def control_fan(power_on):
    # Control USB power using uhubctl
    if power_on:
        print("Turning fan ON")
        os.system(f"sudo uhubctl -l {HUB_ID} -p {PORT_ID} -a on")
    else:
        print("Turning fan OFF")
        os.system(f"sudo uhubctl -l {HUB_ID} -p {PORT_ID} -a off")

fan_on = False
try:
    while True:
        cpu_temp = get_cpu_temp()
        
        # Control fan based on temperature thresholds
        if cpu_temp >= TEMP_ON and not fan_on:
            control_fan(True)
            fan_on = True
        elif cpu_temp <= TEMP_OFF and fan_on:
            control_fan(False)
            fan_on = False

        # Print current status
        print(f"CPU Temp: {cpu_temp}°C, Fan Status: {'On' if fan_on else 'Off'}")
        time.sleep(5)

except KeyboardInterrupt:
    # Ensure the fan is turned off on exit
    control_fan(False)
