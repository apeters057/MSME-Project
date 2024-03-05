import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')

import time
import csv
import threading
import keyboard
from robot_hat import Servo ,Motors
from VoltageRead import VoltageRead_ADS1115 ,VoltageRead_Battery
# Create Motor and Servo object
motors = Motors()
motors.set_left_id(1)
motors.set_right_id(2)



def log_sensor_data(file_path, value):
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
   
    with open(file_path, 'a', newline='') as csv_file:
        # Create a CSV writer object
        csv_writer = csv.writer(csv_file)
        
         # Format data as a list
        data_row = [timestamp, value]
        
         # Write data to the CSV file
        csv_writer.writerow(data_row)
        
def collect_and_log_data(file_path):
    # Create a flag to signal the thread to stop
    exit_flag = threading.Event()
    
    def check_exit():
        # Check for 'Ctrl + Z' key press
        keyboard.wait('ctrl+z')
        print("Data collection ended.")
        # Set the exit flag to stop the main loop
        exit_flag.set()
        
    # Start the thread to check for 'Ctrl + Z' key press
    exit_thread = threading.Thread(target=check_exit)
    exit_thread.start()
    
    try:
        while not exit_flag.is_set():
            # Replace this with your actual data collection logic
#             sensor_value = VoltageRead_ADS1115
            sensor_value = VoltageRead_Battery()

            # Log the collected data to the CSV file
            log_sensor_data(file_path, sensor_value)

            # Optionally, introduce a delay to control the frequency of data collection
            time.sleep(5)  # Adjust the sleep duration as needed

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Ensure the thread is joined before exiting
        exit_thread.join()
    


# Specify the file name and path
file_path = '/home/apeters/project/env/src/Tests/TestData/DataSendTest_Terminal.csv'
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