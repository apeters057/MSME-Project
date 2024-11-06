import sys
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

from pynput import keyboard
import threading
import time
import csv
import mpu6050
from robot_hat import Servo ,Motors
from VoltageRead import VoltageRead_ADS1115 , VoltageRead_Battery
from MPU6050 import accelerometer ,gyroscope ,velocity

## Initialize
StartSignal = 0
motors = Motors()
motors.set_left_id(1) # Setup left and right motors (Drivers POV)
motors.set_right_id(2)
motor_value = 0  #Initial Motor Value
##SteeringAngle  

steer_angle_i = -0
steer_angle = steer_angle_i # Center -- Limits = Right + Left -  [ 30 /-30 degrees (deg)]
servo_Steer = Servo("P2")
servo_Steer.angle(steer_angle)


def log_sensor_data(file_path, elapsed_time, StartSignal, voltage_motor, accel_X, accel_Y,accel_Z, voltage_battery):
    with open(file_path, 'a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        data_row = [elapsed_time, StartSignal, voltage_motor, accel_X, accel_Y,accel_Z, voltage_battery]
        csv_writer.writerow(data_row)

def collect_and_log_data(file_path):
    exit_flag = threading.Event()

    
    def check_exit(key):
        nonlocal exit_flag
        global motor_value
        global steer_angle
        global StartSignal 
        try:
#Drive Various Step inputs via PWM Signals           
            if key == keyboard.KeyCode.from_char('0'):
                motor_value = 0
                StartSignal = 0
                print('Motors = 0 %')
            elif key == keyboard.KeyCode.from_char('5'):
                time.sleep(2)
                StartSignal = 1
                motor_value = 50
                print('Motors = 50 %')
            elif key == keyboard.KeyCode.from_char('7'):
                time.sleep(2)
                motor_value = 70
                StartSignal = 1
                print('Motors = 70%')
            elif key == keyboard.KeyCode.from_char('8'):
                time.sleep(2)
                motor_value = 80
                StartSignal = 1
                print('Motors = 80%')
            elif key == keyboard.KeyCode.from_char('9'):
                time.sleep(2)
                motor_value = 90
                StartSignal = 1
                print('Motors = 90%')
            elif key == keyboard.KeyCode.from_char('1'):
                time.sleep(2)
                motor_value = 100
                StartSignal = 1
                print('Motors = 100%')
            elif key == keyboard.KeyCode.from_char('b'):
                time.sleep(1)
                motor_value = -50
                StartSignal = 0
                print('Reverse Motors = 50 %')
#Steer               
            elif key == keyboard.KeyCode.from_char('q'):
                steer_angle = steer_angle_i 
            elif key == keyboard.Key.left:
                # Turn car Left (up to 30)
                steer_angle = min(30, steer_angle - 2)
                time.sleep(0.001)
            elif key == keyboard.Key.right:
                # Turn car Right (up to 30)
                steer_angle = max(-30, steer_angle + 2)
                time.sleep(0.001)
#Exit
            elif key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
                exit_flag.set()
                motors.stop()
                print("Motors stopped")
                
        except Exception as e:
            print(f"Error in check_exit: {e}")
#         finally:
#             motors.stop()
#             
    def on_press(key):
        check_exit(key)

    exit_thread = threading.Thread(target=lambda: keyboard.Listener(on_press=on_press).start())
    exit_thread.start()

    try:
        start_time = time.time()
        while not exit_flag.is_set():
            # Set up Motor Direction Speed
            # Motor 1 clockwise at 100% speed
            motors[1].speed(motor_value) #Left
            motors[2].speed(-motor_value) #Right
            servo_Steer.angle(steer_angle) #Steer
            # Replace this with your actual data collection logic
#             velocity_data = velocity()
#             velocity_x = velocity_data[0]
#             velocity_y = velocity_data[1]
#             velocity_z = velocity_data[2]
            accel_data = accelerometer()
            accel_X = accel_data[0]
            accel_Y = accel_data[1]
            accel_Z = accel_data[2]
            
            voltage_motor = VoltageRead_ADS1115()
            voltage_battery = VoltageRead_Battery()
            end_time = time.time()
            elapsed_time = end_time - start_time
            time_step = elapsed_time
            # Log the collected data to the CSV file
            log_sensor_data(file_path,elapsed_time, StartSignal, voltage_motor, accel_X, accel_Y,accel_Z, voltage_battery)
            # Optionally, introduce a delay to control the frequency of data collection
            # time.sleep(0.01)  # Adjust the sleep duration as needed
            
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Ensure the thread is joined before exiting
        exit_thread.join()

# Specify the file name and path
file_path = '/home/apeters/project/env/src/Tests/TestData/105_Capacitor/VOID.csv'
collect_and_log_data(file_path)

print(f'Data has been logged to {file_path}')