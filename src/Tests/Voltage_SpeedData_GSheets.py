import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')

import time
import csv
import threading
import keyboard
import gspread
from oauth2client.service_account import ServiceAccountCredentials
from robot_hat import Servo ,Motors
from VoltageRead import VoltageRead_ADS1115 ,VoltageRead_Battery

##### Google Sheets

# Set the scope and credentials
scope = ['https://spreadsheets.google.com/feeds', 'https://www.googleapis.com/auth/drive']
credentials = ServiceAccountCredentials.from_json_keyfile_name('/home/apeters/project/env/src/Tests/credentials.json', scope)
gc = gspread.authorize(credentials)
# Open the Google Spreadsheet using its title
spreadsheet = gc.open('Voltage_SpeedData')
# Select the worksheet
worksheet = spreadsheet.sheet1
# Update cell values (Row , Column, Value)
# Title Cells
worksheet.update_cell(1, 1, 'Time')
worksheet.update_cell(1, 2, 'Input - Voltage')
worksheet.update_cell(1, 3, 'Output - Speed')

# Get the next available row in the sheet
next_row = len(worksheet.get_all_values()) + 1
print('Data sent to Google Sheets!')

###
# Create Motor and Servo object
motors = Motors()
motors.set_left_id(1)
motors.set_right_id(2)

timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
sample_time = 1  # Define the sample time in seconds
start_time = time.time()
while True:
# Write data to the sheet
    
    # Your data collection or processing logic goes here
    Input_Voltage = VoltageRead_ADS1115()
    Output_Speed = VoltageRead_Battery()
    end_time = time.time()
    elapsed_time = end_time - start_time
    
    # Calculate the remaining time to achieve the desired sample time
    data_to_write = [elapsed_time, Input_Voltage , Output_Speed ]
    worksheet.insert_row(data_to_write, index=next_row)
    # Introduce a delay to achieve the desired sample time
    





