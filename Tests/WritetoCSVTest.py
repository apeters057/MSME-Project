import sys

sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

import time
import csv
import threading
import keyboard
import mpu6050
from robot_hat import Servo ,Motors
from VoltageRead import VoltageRead_ADS1115 , VoltageRead_Battery
from MPU6050 import accelerometer ,gyroscope ,velocity 
# Create Motor and Servo object
motors = Motors()
# Setup left and right motors (Drivers POV)?? i dont think this does anything tbh
motors.set_left_id(1)
motors.set_right_id(2)

def log_sensor_data(file_path,elapsed_time, value1, value2, value3):
    #timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    
    with open(file_path, 'a', newline='') as csv_file:
        # Create a CSV writer object
        csv_writer = csv.writer(csv_file)
        
         # Format data as a list
        data_row = [elapsed_time, value1, value2, value3]
        
         # Write data to the CSV file
        csv_writer.writerow(data_row)
        
def collect_and_log_data(file_path):
    # Create a flag to signal the thread to stop
    exit_flag = threading.Event()
    
    def check_exit(key):
        try:
        # Check for 'Ctrl + Z' key press
            if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
            
    #        keyboard.wait('ctrl+c')
    #        print("Data collection ended.")
                motors.stop()
            # Set the exit flag to stop the main loop
                exit_flag.set()
        except Exception as e:
            print(f"Error: {e}")
        
    # Start the thread to check for 'Ctrl + Z' key press
    exit_thread = threading.Thread(target=check_exit)
    exit_thread.start()
    
    try:
        start_time = time.time()
        while not exit_flag.is_set():
            #Set up Motor Direction Speed
            # Motor 1 clockwise at 100% speed
            motors[1].speed(100) #Left
            # Motor 2 clockwise at 100% speed
            motors[2].speed(-100) #Right
            # Replace this with your actual data collection logic
            VelocityData = velocity()
            Velocity_X = VelocityData[0]
            Velocity_Y = VelocityData[1]
            Velocity_Z = VelocityData[2]
            
            
            value1 = VoltageRead_ADS1115()
            value2 = Velocity_X
            value3 = VoltageRead_Battery()
            end_time = time.time()
            elapsed_time = end_time - start_time
            # Log the collected data to the CSV file
            log_sensor_data(file_path,elapsed_time, value1,value2,value3)
            # Optionally, introduce a delay to control the frequency of data collection
            time.sleep(0.1)  # Adjust the sleep duration as needed
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Ensure the thread is joined before exiting
        exit_thread.join()
    
# Specify the file name and path
file_path = '/home/apeters/project/env/src/Tests/TestData/Test1_UnitStep_PWM100.csv'
collect_and_log_data(file_path)

print(f'Data has been logged to {file_path}')

    
# while True:
#     
#     #Set up Motor Direction Speed
#     motors[1].speed(90) #Left
#     motors[2].speed(0) #Left
#     motors.stop()                                                                                               
#     print(VoltageRead_ADS1115())
#     print(VoltageRead_Battery())
#     time.sleep(1)