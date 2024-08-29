import mpu6050
import time
import numpy as np

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)
# velocity_data = [0] # Intitial Velocity is 0
# Define a function to read the sensor data
def accelerometer():
    # Read the accelerometer values
    accel_data = mpu6050.get_accel_data()
    accel_X = accel_data['x']
    accel_Y = accel_data['y']
    accel_Z = accel_data['z']
    # Read temp
    #temperature = mpu6050.get_temp()
    return accel_X ,accel_Y ,accel_Z

def gyroscope():
    
    gyro_data = mpu6050.get_gyro_data()
    gyro_X = gyro_data['x']
    gyro_Y = gyro_data['y']
    gyro_Z = gyro_data['z']
    return gyro_X ,gyro_Y ,gyro_Z

def velocity():

    accel_data = mpu6050.get_accel_data()
    accel_X = accel_data['x']
    accel_Y = accel_data['y']
    accel_Z = accel_data['z']
    
    # Calculate velocity using cumulative sum
    initial_velocity = 0
    delta_t = 0.01  # Adjust this value based on your actual time interval
    Velocity_X = float(initial_velocity + np.cumsum(accel_X * delta_t)) # 0
    Velocity_Y = float(initial_velocity + np.cumsum(accel_Y * delta_t)) # 1
    Velocity_Z = float(initial_velocity + np.cumsum(accel_Z * delta_t))# 2
    time.sleep(delta_t)  # Adjust the delay as needed for sampling rate
    return float(Velocity_X), Velocity_Y , Velocity_Z

#while True:
#    accel_data = accelerometer()
 #   print("Accelerometer Data:", accel_data)
 # vel_data = velocity()
  #print("Velocity Data:", vel_data[0])